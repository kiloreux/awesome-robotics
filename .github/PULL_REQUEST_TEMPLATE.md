## Description
<!-- Describe your changes in detail. Include what problem it solves and how it solves it. -->

## Motivation and Context
<!-- Why is this change required? What issue does it resolve? -->
<!-- If it fixes an open issue, please link to the issue here. -->

## Safety & Sandbox
<!-- For any tool that writes to ROS2 (topics, parameters, nodes): -->
- [ ] This tool respects the `CommandSandbox` (checks `check_publish` / `check_set_parameter`).
- [ ] This change maintains the strict `SAFETY_LEVEL=safe_write` default.

## Testing
<!-- Describe how you tested your changes (e.g., simulation mode, on a real Jetson). -->
- [ ] All tests pass locally (`python run_tests.py`).
- [ ] I have added new tests covering my changes.
- [ ] My code passes the `ruff` linter checks.

## Types of changes
- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to change)
- [ ] Documentation update
