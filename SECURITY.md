# Security Policy

## Threat Model: AI & Physical Hardware

Bridging an autonomous AI agent to physical robotics introduces unique attack vectors:
1. **Hallucinated Commands**: The LLM predicts a valid-looking but physically catastrophic command (e.g., publishing `velocity = 100 m/s`).
2. **Infinite Loops**: The LLM traps itself in a read-publish loop.
3. **Malicious Payloads**: Injection of malformed ROS2 parameters leading to buffer overflows in C++ nodes.

## The Command Sandbox

To mitigate this, `ros2-mcp-server` enforces a strict permission boundary.
We implement three safety levels via the `SAFETY_LEVEL` environment variable:

- **`read_only` (Default)**: The AI can list nodes, read topics, and fetch diagnostics, but it **cannot** publish, tune PIDs, or alter parameters.
- **`safe_write`**: The AI can read data and invoke safely allowlisted tuning tools (like PID tuning or parameter adjustment on safe namespaces). Direct publishing to actuator nodes is blocked.
- **`full`**: Unrestricted access. *Only use this in a simulated environment.*

## Reporting a Vulnerability

If you discover a sandbox escape or a crash loop vulnerability, please do **NOT** create a public GitHub issue.
Instead, email us at `abz.king.1.9.2003@gmail.com` with the subject `[SECURITY] ROS2 MCP Server`. We will respond within 48 hours.
