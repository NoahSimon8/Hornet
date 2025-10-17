# tools/start_live_plot.py
Import("env")
if env.IsIntegrationDump(): Return()

import os, sys, subprocess

def _sanitize(value):
    if not value:
        return None
    value = str(value).strip()
    if not value or value.lower() == "none":
        return None
    if value.startswith("${") and value.endswith("}"):
        return None
    return value

def _is_placeholder(value):
    if not value:
        return True
    canon = value.strip().lower()
    return canon in {"auto", "none", "monitor_port", "upload_port", "comx"}

def _get_project_option(env, name):
    try:
        return env.GetProjectOption(name)
    except Exception:
        return None

def _resolve_port(env):
    candidates = [
        _get_project_option(env, "monitor_port"),
        _get_project_option(env, "upload_port"),
    ]
    for key in ("MONITOR_PORT", "UPLOAD_PORT"):
        try:
            candidates.append(env[key])
        except KeyError:
            pass
        if hasattr(env, "subst"):
            candidates.append(env.subst(f"${key}"))
    for value in candidates:
        value = _sanitize(value)
        if value and not _is_placeholder(value):
            return value
    if hasattr(env, "AutodetectUploadPort"):
        try:
            value = env.AutodetectUploadPort()
            value = _sanitize(value)
            if value and not _is_placeholder(value):
                print(f"Auto-detected upload port: {value}")
                return value
        except Exception as exc:
            print(f"Auto-detect upload port failed: {exc}")
    return "COM3" if os.name == "nt" else "/dev/ttyACM0"

def start_viewer(source, target, env, **kwargs):
    project_dir = env["PROJECT_DIR"]
    candidate_scripts = [
        os.path.join(project_dir, "scripts", "live_plot.py"),
        os.path.join(project_dir, "tools", "live_plot.py"),
    ]
    viewer = next((p for p in candidate_scripts if os.path.isfile(p)), candidate_scripts[0])
    port = _resolve_port(env)
    baud = env.GetProjectOption("monitor_speed") or "2000000"
    build_dir = env.subst("${BUILD_DIR}") if hasattr(env, "subst") else os.path.join(project_dir, ".pio", "build")
    log_path = os.path.join(build_dir, "live_plot.log")
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    cmd = [sys.executable, viewer, "--port", port, "--baud", str(baud), "--log", log_path]
    # Detach so PlatformIO finishes
    launch_kwargs = {}
    if os.name == "nt":
        launch_kwargs["creationflags"] = 0x00000008  # CREATE_NEW_CONSOLE
    else:
        launch_kwargs["start_new_session"] = True
    proc_env = os.environ.copy()
    proc_env.setdefault("PYTHONUNBUFFERED", "1")
    try:
        subprocess.Popen(cmd, env=proc_env, **launch_kwargs)
        print(f"Started live_plot.py: {' '.join(cmd)}")
        print(f"Viewer serial port: {port} @ {baud}")
        print(f"Python interpreter: {sys.executable}")
        print(f"Live-plot log: {log_path}")
    except Exception as exc:
        print(f"Failed to launch live_plot.py: {exc}")

env.AddPostAction("upload", start_viewer)
# Optional convenience target: pio run -t view
env.AddCustomTarget("view", None, [start_viewer], title="Start Live Viewer")
