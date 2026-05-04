# common

Shared demo-owned runtime configuration lives here.

Use these YAML files as the primary tuning surface for:
- drive scaling
- joystick mapping and filtering
- replay supervision
- path following
- WILN replay behavior

Per-demo files under `demos/*/config/` should only contain:
- launch toggles
- hardware paths
- demo-specific record options

Do not tune runtime behavior in `src/**/config` during normal operation.
