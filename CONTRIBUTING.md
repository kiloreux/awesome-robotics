# Contributing to `ros2-mcp-server`

First off, thank you for considering contributing to `ros2-mcp-server`! We welcome all contributions, from bug reports and documentation fixes to major feature additions.

## Local Development & Instant Playground

You do **not** need a physical robot or a complex ROS2 installation to develop for this repository.
We have built a robust simulation layer that generates synthetic LiDAR, IMU, and Battery data.

To test your code locally:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -e .
python ros2_mcp/server.py --demo-sim
```

## Pull Request Process

1. **Fork & Branch**: Fork the repository and create a descriptive branch name (e.g. `feat/new-mcp-tool`).
2. **Strict Linting**: We use `ruff` to ensure absolute codebase integrity. Before committing, run:
   ```bash
   python -m ruff check . --fix
   python -m ruff format .
   ```
3. **Tests**: Ensure the 38-test suite passes via `python run_tests.py`.
4. **Submit PR**: Use our provided PR template to explain your changes. We will review your PR within 2-3 business days.
