/**
 * Fault events: paged, ordered and keeping every occurrence. A tick with more
 * than one event keeps all of them; the marker groups by detection time but the
 * event list never collapses the group. Clicking an event asks the shared time
 * model to seek; this view never moves a chart directly.
 */

import { Us } from './api.js';
import { RelativeTime } from './time_display.js';

function ReasonName(event) {
  return event.reason_name ?? event.condition_name ?? `reason ${event.reason}`;
}

export class EventsView {
  constructor(callbacks = {}) {
    this.callbacks = callbacks;
    this.timeOriginUs = 0n;
    this.events = [];
    this.markers = [];
    this.integrity = null;
    this.continuation = null;
    this.selected = null;
  }

  Reset(originUs = this.timeOriginUs) {
    this.timeOriginUs = originUs;
    this.events = [];
    this.markers = [];
    this.integrity = null;
    this.continuation = null;
    this.selected = null;
  }

  Append(payload) {
    if (payload?.integrity) {
      this.integrity = payload.integrity;
    }
    const byId = new Map(
      this.events.map((event, index) => [event.event_id, index]),
    );
    for (const event of payload?.events ?? []) {
      const index = byId.get(event.event_id);
      if (index === undefined) {
        byId.set(event.event_id, this.events.length);
        this.events.push(event);
      } else if (payload.integrity) {
        // A committed row supersedes its live copy, without clearing the tail.
        this.events[index] = event;
        if (this.selected?.event_id === event.event_id) this.selected = event;
      }
    }
    if (payload && Object.hasOwn(payload, 'continuation')) {
      this.continuation = payload.continuation;
    }
    this.BuildMarkers();
  }

  BuildMarkers() {
    const grouped = new Map();
    for (const event of this.events) {
      const key = String(event.detected_stamp_us);
      if (!grouped.has(key)) {
        grouped.set(key, {
          timeUs: Us(event.detected_stamp_us),
          eventIds: [],
        });
      }
      grouped.get(key).eventIds.push(event.event_id);
    }
    this.markers = [...grouped.values()].sort((left, right) =>
      left.timeUs < right.timeUs ? -1 : left.timeUs > right.timeUs ? 1 : 0,
    );
  }

  Select(eventId) {
    const event =
      this.events.find((entry) => entry.event_id === eventId) ?? null;
    this.selected = event;
    if (event) {
      this.callbacks.onSelect?.(event);
      this.callbacks.onSeek?.(Us(event.detected_stamp_us), event);
    }
    return event;
  }

  Summary() {
    if (!this.integrity) {
      return '';
    }
    const faults = this.integrity.faults ?? this.integrity;
    return [
      faults.complete === false ? 'fault stream open' : 'fault stream complete',
      faults.truncated ? 'truncated' : '',
      faults.corrupt ? 'corrupt' : '',
      faults.losses?.length ? `${faults.losses.length} loss span(s)` : '',
    ]
      .filter(Boolean)
      .join(' · ');
  }

  FaultNotice(joints) {
    let latestId = 0n;
    let firstStamp = 0n;
    const details = new Set();
    for (const event of this.events) {
      const id = Us(event.fault_id ?? '0');
      if (id === 0n || id < latestId) continue;
      const stamp = Us(event.detected_stamp_us);
      // Fault IDs increase per runtime. Later observations from the same
      // latch must not replace the reasons seen at its first detection.
      if (id > latestId || stamp < firstStamp) {
        latestId = id;
        firstStamp = stamp;
        details.clear();
      }
      if (stamp !== firstStamp) continue;
      const joint = event.joint_name ?? joints?.[event.joint_index];
      const reason = ReasonName(event);
      details.add(joint ? `${reason} (${joint})` : reason);
    }
    return details.size
      ? `Fault observed: ${[...details].join(', ')} · t=${RelativeTime(firstStamp, this.timeOriginUs)}`
      : '';
  }

  Render(container) {
    if (!container || !container.ownerDocument) {
      return;
    }
    const document = container.ownerDocument;
    container.replaceChildren();
    for (const event of this.events) {
      const item = document.createElement('button');
      item.type = 'button';
      item.className = 'event-item';
      item.dataset.eventId = event.event_id;
      item.dataset.stamp = event.detected_stamp_us;
      if (this.selected?.event_id === event.event_id) {
        item.classList.add('selected');
      }
      const time = document.createElement('span');
      time.className = 'event-time';
      time.textContent = `t=${RelativeTime(event.detected_stamp_us, this.timeOriginUs)}`;
      const reason = document.createElement('span');
      reason.className = 'event-reason';
      reason.textContent = ReasonName(event);
      const message = document.createElement('span');
      message.className = 'event-message';
      message.textContent = event.message ?? '';
      item.append(time, reason, message);
      item.addEventListener('click', () => {
        this.Select(event.event_id);
        this.Render(container);
      });
      container.append(item);
    }
    if (this.continuation) {
      const more = document.createElement('button');
      more.type = 'button';
      more.className = 'event-more';
      more.textContent = 'Load more events';
      more.addEventListener('click', () =>
        this.callbacks.onMore?.(this.continuation),
      );
      container.append(more);
    }
  }

  DetailText() {
    const event = this.selected;
    if (!event) {
      return '';
    }
    return [
      `event ${event.event_id}`,
      `detected ${RelativeTime(event.detected_stamp_us, this.timeOriginUs)} · sample ${RelativeTime(event.sample_stamp_us, this.timeOriginUs)}`,
      `reason ${ReasonName(event)} (${event.reason})`,
      `condition ${event.condition_name} (${event.condition})`,
      `joint ${event.joint_name ?? event.joint_index ?? '—'}`,
      `error ${event.error_code}`,
      `observed ${String(event.observed_value)} · limit ${String(event.limit)}`,
      `output ${event.output_action_name} (${event.output_action})`,
      `message ${event.message}`,
      `message_hex ${event.message_hex}`,
    ].join('\n');
  }
}
