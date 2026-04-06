# AGENTS

When working on anything related to the Webots Python API, please consult the local docs bundle before making assumptions.

Primary references:

- `docs/webots-python-llms-full.txt`
- `docs/generate_webots_python_llms_full.py`

Light guidance:

- Use `docs/webots-python-llms-full.txt` as the first-stop reference for Webots Python controller code, device APIs, and supervisor APIs.
- Prefer matching Webots class names, method names, and controller patterns to the docs rather than relying on memory.
- If Webots Python behavior is unclear, check the bundled docs before inventing wrappers or helper abstractions.
- If the docs bundle seems outdated, regenerate it with `python3 docs/generate_webots_python_llms_full.py` and then continue but this is mostly unlikely.
