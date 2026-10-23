#!/usr/bin/env python3
"""The generated SDK revision, proven through the real module path.

Usage: build_identity_test.py <source directory>

A temporary minimal Git repository includes cmake/BuildIdentity.cmake and
builds a small library plus a probe against it, so the test exercises the same
generated source, add_dependencies path and library the two processes link -
not a reimplementation of the generator. It then proves:

  * building a fresh repository embeds commit A;
  * building again with the same SHA leaves the generated file untouched;
  * after commit B, a build without reconfiguring updates the embedded SHA;
  * the source repository's HEAD is never changed;
  * a source copy whose .git was removed embeds "unknown".

Every temporary directory is removed in the finally block.
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

FAILURES = []
SHA_PATTERN = re.compile(r'constexpr char kSdkGitSha\[\] = "([^"]*)";')

FIXTURE_CMAKE = """\
cmake_minimum_required(VERSION 3.22)
project(irmv_build_identity_fixture LANGUAGES CXX)
include("{module}")
irmv_add_build_identity(irmv_build_identity)
add_executable(identity_probe main.cpp)
target_compile_features(identity_probe PRIVATE cxx_std_17)
target_link_libraries(identity_probe PRIVATE irmv_build_identity)
"""

FIXTURE_MAIN = """\
#include <cstdio>
#include <string_view>

#include "irmv/build_identity.hpp"

int main() {
  const std::string_view sha = irmv::SdkGitSha();
  std::printf("%.*s\\n", static_cast<int>(sha.size()), sha.data());
  return 0;
}
"""


def check(condition, message):
    if condition:
        return True
    FAILURES.append(message)
    print("FAIL: " + message)
    return False


def run(command, cwd=None, env=None):
    completed = subprocess.run(
        [str(item) for item in command],
        cwd=str(cwd) if cwd else None,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if completed.returncode != 0:
        print("command failed: " + " ".join(str(item) for item in command))
        print(completed.stdout)
    return completed


def git(repo, *arguments, env=None):
    return run(["git", "-C", repo, *arguments], env=env)


def head(repo, env=None):
    completed = git(repo, "rev-parse", "HEAD", env=env)
    check(completed.returncode == 0, "git rev-parse HEAD in " + str(repo))
    return completed.stdout.strip()


def commit(repo, text, message, env):
    (repo / "tracked.txt").write_text(text, encoding="utf-8")
    added = git(repo, "add", "tracked.txt", env=env)
    check(added.returncode == 0, "git add in the fixture repository")
    committed = git(repo, "-c", "commit.gpgsign=false", "commit", "-m",
                    message, env=env)
    check(committed.returncode == 0, "git commit " + message)
    return head(repo, env=env)


def generated_sha(build_dir):
    generated = build_dir / "generated" / "build_identity.cpp"
    if not generated.is_file():
        return None
    match = SHA_PATTERN.search(generated.read_text(encoding="utf-8"))
    return match.group(1) if match else None


def probe_sha(build_dir):
    completed = run([build_dir / "identity_probe"])
    return completed.stdout.strip() if completed.returncode == 0 else None


def configure_and_build(source, build, env):
    configured = run(
        ["cmake", "-S", source, "-B", build, "-DCMAKE_BUILD_TYPE=Debug"],
        env=env,
    )
    if not check(configured.returncode == 0, "configure " + str(source)):
        return False
    built = run(["cmake", "--build", build, "-j2"], env=env)
    return check(built.returncode == 0, "build " + str(build))


def write_fixture(source, module):
    (source / "include" / "irmv").mkdir(parents=True, exist_ok=True)
    (source / "CMakeLists.txt").write_text(
        FIXTURE_CMAKE.format(module=module.as_posix()), encoding="utf-8"
    )
    (source / "main.cpp").write_text(FIXTURE_MAIN, encoding="utf-8")


def main():
    source_root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    module = source_root / "cmake" / "BuildIdentity.cmake"
    header = source_root / "include" / "irmv" / "build_identity.hpp"
    if not module.is_file():
        print("FAIL: missing generator module " + str(module))
        return 1
    if not header.is_file():
        print("FAIL: missing header " + str(header))
        return 1

    original_head = head(source_root)
    environment = os.environ.copy()
    # Keep a copied source without .git from discovering a repository above the
    # temporary root; the generator must report unknown regardless.
    temporary_root = Path(tempfile.mkdtemp(prefix="irmv-build-identity-"))
    environment["GIT_CEILING_DIRECTORIES"] = str(temporary_root)

    try:
        repository = temporary_root / "repository"
        repository.mkdir()
        initialized = run(["git", "init", "-q", repository], env=environment)
        check(initialized.returncode == 0, "git init in the temporary root")
        check(
            git(repository, "config", "user.name", "Build Identity Test",
                env=environment).returncode == 0,
            "configure the fixture user name",
        )
        check(
            git(repository, "config", "user.email", "identity@example.invalid",
                env=environment).returncode == 0,
            "configure the fixture user email",
        )

        write_fixture(repository, module)
        shutil.copyfile(header, repository / "include" / "irmv" /
                        "build_identity.hpp")

        sha_a = commit(repository, "A\n", "A", environment)
        build = temporary_root / "build"
        if not configure_and_build(repository, build, environment):
            return 1
        check(generated_sha(build) == sha_a,
              "the generated source carries commit A")
        check(probe_sha(build) == sha_a,
              "the linked probe reports commit A")

        # A build with unchanged HEAD must not rewrite the generated source.
        generated = build / "generated" / "build_identity.cpp"
        before = generated.stat().st_mtime_ns
        time.sleep(0.02)
        check(
            run(["cmake", "--build", build, "-j2"],
                env=environment).returncode == 0,
            "rebuild with the same HEAD",
        )
        check(generated.stat().st_mtime_ns == before,
              "an unchanged SHA leaves the generated file's mtime untouched")
        check(generated_sha(build) == sha_a,
              "an unchanged SHA leaves the embedded revision at A")

        # A new commit reaches the generated source through the build alone:
        # no reconfigure is run between the two builds.
        sha_b = commit(repository, "B\n", "B", environment)
        check(sha_b != sha_a, "commit B differs from commit A")
        check(generated_sha(build) == sha_a,
              "the commit alone does not rewrite the generated source")
        check(
            run(["cmake", "--build", build, "-j2"],
                env=environment).returncode == 0,
            "build after commit B",
        )
        check(generated_sha(build) == sha_b,
              "the next build picks commit B up")
        check(probe_sha(build) == sha_b,
              "the relinked probe reports commit B")

        # The generator must not touch this repository's HEAD.
        check(head(source_root) == original_head,
              "the source repository's HEAD is unchanged")

        # A copied source with no .git is not a repository. It is put inside
        # the fixture repository, so Git would report that parent as the
        # toplevel and only the generator's own-repository guard yields
        # unknown. Copy outside first: copying a tree into itself would recurse.
        staged_no_git = temporary_root / "no-git-staged"
        shutil.copytree(
            repository,
            staged_no_git,
            ignore=shutil.ignore_patterns(".git"),
        )
        no_git = repository / "no-git"
        shutil.move(str(staged_no_git), str(no_git))
        parent = git(no_git, "rev-parse", "--show-toplevel", env=environment)
        check(parent.returncode == 0,
              "Git reports the fixture repository from the no-Git copy")
        check(Path(parent.stdout.strip()).resolve() == repository.resolve(),
              "the no-Git copy is inside the fixture repository")
        no_git_build = temporary_root / "no-git-build"
        if not configure_and_build(no_git, no_git_build, environment):
            return 1
        check(generated_sha(no_git_build) == "unknown",
              "a source without .git embeds unknown")
        check(probe_sha(no_git_build) == "unknown",
              "the probe of a source without .git reports unknown")
        check(head(source_root) == original_head,
              "the no-Git copy leaves this repository's HEAD unchanged")
    finally:
        shutil.rmtree(temporary_root, ignore_errors=True)

    if FAILURES:
        print("%d check(s) failed" % len(FAILURES))
        return 1
    print("build_identity_test: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
