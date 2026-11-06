/**
 * Shared type documentation for the workbench. These are JSDoc typedefs, not
 * runtime values; the models are plain JSON so Node tests and the browser use
 * exactly the same modules.
 *
 * @typedef {Object} Signal
 * @property {string} id Canonical source/joint/field identity
 * @property {'runtime'|'application'} source
 * @property {string} joint_name Empty for a non-joint signal
 * @property {string} field
 * @property {string} label
 * @property {string} unit
 * @property {string} quantity
 * @property {'scalar'|'bool'|'enum'} value_kind
 * @property {string} validity
 *
 * @typedef {Object} Axis
 * @property {string} id
 * @property {string} quantity
 * @property {string} unit
 * @property {[number, number]|null} [range]
 *
 * @typedef {Object} Curve
 * @property {string} signalId
 * @property {string} axisId
 * @property {string} style
 *
 * @typedef {Object} Chart
 * @property {string} id
 * @property {string} title
 * @property {Curve[]} curves
 * @property {Axis[]} axes
 *
 * @typedef {{type:'empty'}} EmptyDock
 * @typedef {{type:'leaf', chartId:string}} LeafDock
 * @typedef {{type:'split', direction:'row'|'column', ratio:number, first:DockNode, second:DockNode}} SplitDock
 * @typedef {EmptyDock|LeafDock|SplitDock} DockNode
 *
 * @typedef {Object} Layout
 * @property {1} version
 * @property {string} name
 * @property {DockNode} dock
 * @property {Chart[]} charts
 */

export {};
