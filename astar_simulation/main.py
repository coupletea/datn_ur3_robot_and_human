from __future__ import annotations

import sys
from pathlib import Path


def _ensure_repo_on_path() -> None:
    repo_root = str(Path(__file__).resolve().parents[1])
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)


def main() -> int:
    _ensure_repo_on_path()
    try:
        from PySide6 import QtWidgets
        from astar_simulation.app_window import AppWindow
    except ImportError as exc:
        print(
            "Missing desktop dependency. Run:\n"
            "  python -m pip install -r astar_simulation/requirements.txt\n"
            f"\nImport error: {exc}",
            file=sys.stderr,
        )
        return 1

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    window = AppWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
