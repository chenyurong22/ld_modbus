# Contributing

Keep the portable core C99, bounded, transport-independent, and free of dynamic
allocation. Public APIs require concise comments and caller-owned storage.

Before opening a pull request, run:

```powershell
.\scripts\check_quality.ps1
.\scripts\test_mingw.ps1
```

Add host tests for every behavior change, including malformed input and boundary
conditions. Hardware-specific adapters belong outside `src/` and `include/`.
