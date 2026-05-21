#!/usr/bin/env python3

import os
import shutil
import signal
import subprocess
import time
from datetime import datetime
from fractions import Fraction

import rospy


def _bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _framerate(value):
    if isinstance(value, str) and "/" in value:
        numerator, denominator = value.split("/", 1)
        return f"{int(numerator)}/{int(denominator)}"
    fraction = Fraction(float(value)).limit_denominator(1001)
    return f"{fraction.numerator}/{fraction.denominator}"


def _v4l2_pipeline(params):
    return [
        "v4l2src",
        f"device={params['device']}",
        "!",
        f"video/x-raw,format={params['input_format']},width={params['width']},height={params['height']},framerate={_framerate(params['fps'])}",
        "!",
        "queue",
        "leaky=downstream",
        "max-size-buffers=4",
        "!",
        "nvvidconv",
        "!",
        "video/x-raw(memory:NVMM),format=NV12",
    ]


def _csi_pipeline(params):
    return [
        "nvarguscamerasrc",
        f"sensor-id={params['sensor_id']}",
        "!",
        f"video/x-raw(memory:NVMM),format=NV12,width={params['width']},height={params['height']},framerate={_framerate(params['fps'])}",
        "!",
        "queue",
        "leaky=downstream",
        "max-size-buffers=4",
        "!",
        "nvvidconv",
        f"flip-method={params['flip_method']}",
        "!",
        "video/x-raw(memory:NVMM),format=NV12",
    ]


def _recording_pipeline(params, output_path):
    source_type = params["source_type"].lower()
    if source_type == "v4l2":
        pipeline = _v4l2_pipeline(params)
    elif source_type == "csi":
        pipeline = _csi_pipeline(params)
    else:
        raise ValueError("~source_type must be 'v4l2' or 'csi'")

    pipeline.extend(
        [
            "!",
            "nvv4l2h265enc",
            f"bitrate={params['bitrate']}",
            f"control-rate={params['control_rate']}",
            f"preset-level={params['preset_level']}",
            f"iframeinterval={params['iframe_interval']}",
            "maxperf-enable=1",
            "!",
            "h265parse",
            "!",
            "queue",
            "max-size-buffers=0",
            "max-size-bytes=8388608",
            "max-size-time=0",
            "!",
            "matroskamux",
            "!",
            "filesink",
            f"location={output_path}",
            "sync=false",
        ]
    )
    return pipeline


def _command_prefix(params):
    prefix = []

    cpu_affinity = params["cpu_affinity"].strip()
    if cpu_affinity:
        if shutil.which("taskset"):
            prefix.extend(["taskset", "-c", cpu_affinity])
        else:
            rospy.logwarn("~cpu_affinity was set, but taskset is not installed")

    if params["nice"] != 0:
        if shutil.which("nice"):
            prefix.extend(["nice", "-n", str(params["nice"])])
        else:
            rospy.logwarn("~nice was set, but nice is not installed")

    if params["ionice"]:
        if shutil.which("ionice"):
            prefix.extend(
                [
                    "ionice",
                    "-c",
                    str(params["ionice_class"]),
                    "-n",
                    str(params["ionice_level"]),
                ]
            )
        else:
            rospy.logwarn("~ionice is true, but ionice is not installed")

    return prefix


def _load_params():
    return {
        "source_type": rospy.get_param("~source_type", "csi"),
        "device": rospy.get_param("~device", "/dev/video0"),
        "sensor_id": int(rospy.get_param("~sensor_id", 0)),
        "output_dir": os.path.expanduser(
            rospy.get_param("~output_dir", "~/recordings/fisheye")
        ),
        "output_mount": rospy.get_param("~output_mount", "/"),
        "require_output_mount": _bool_param("~require_output_mount", True),
        "require_nvme_device": _bool_param("~require_nvme_device", True),
        "output_prefix": rospy.get_param("~output_prefix", "fisheye"),
        "width": int(rospy.get_param("~width", 1280)),
        "height": int(rospy.get_param("~height", 720)),
        "fps": rospy.get_param("~fps", 30.0),
        "input_format": rospy.get_param("~input_format", "UYVY"),
        "flip_method": int(rospy.get_param("~flip_method", 2)),
        "bitrate": int(rospy.get_param("~bitrate", 8000000)),
        "control_rate": int(rospy.get_param("~control_rate", 1)),
        "preset_level": int(rospy.get_param("~preset_level", 1)),
        "iframe_interval": int(rospy.get_param("~iframe_interval", 60)),
        "cpu_affinity": rospy.get_param("~cpu_affinity", ""),
        "nice": int(rospy.get_param("~nice", 5)),
        "ionice": _bool_param("~ionice", True),
        "ionice_class": int(rospy.get_param("~ionice_class", 2)),
        "ionice_level": int(rospy.get_param("~ionice_level", 7)),
        "shutdown_timeout": float(rospy.get_param("~shutdown_timeout", 8.0)),
    }


def _mount_source(path):
    if not shutil.which("findmnt"):
        raise RuntimeError("findmnt is not installed; cannot verify recording filesystem")
    return subprocess.check_output(
        ["findmnt", "-n", "-T", path, "-o", "SOURCE"],
        text=True,
    ).strip()


def _validate_output_storage(params):
    if params["require_output_mount"] and not os.path.ismount(params["output_mount"]):
        raise RuntimeError(f"{params['output_mount']} is not mounted")

    if not params["require_nvme_device"]:
        return

    source = _mount_source(params["output_dir"])
    if "nvme" not in os.path.basename(source):
        raise RuntimeError(
            f"{params['output_dir']} is on {source}, not an NVMe-backed filesystem"
        )
    rospy.loginfo("Recording filesystem is backed by %s", source)


def _make_output_path(params):
    os.makedirs(params["output_dir"], exist_ok=True)
    _validate_output_storage(params)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(params["output_dir"], f"{params['output_prefix']}_{timestamp}.mkv")


def _terminate(process, timeout):
    if process.poll() is not None:
        return

    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout)
        return
    except subprocess.TimeoutExpired:
        rospy.logwarn("GStreamer did not stop after SIGINT; sending SIGTERM")

    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=3.0)
        return
    except subprocess.TimeoutExpired:
        rospy.logerr("GStreamer did not stop after SIGTERM; sending SIGKILL")

    os.killpg(process.pid, signal.SIGKILL)
    process.wait()


def main():
    rospy.init_node("fisheye_recorder")

    if not shutil.which("gst-launch-1.0"):
        rospy.logerr("gst-launch-1.0 is not installed or not in PATH")
        return

    params = _load_params()
    try:
        output_path = _make_output_path(params)
    except OSError as exc:
        rospy.logerr("Could not create output directory %s: %s", params["output_dir"], exc)
        return
    except RuntimeError as exc:
        rospy.logerr("%s", exc)
        return

    pipeline = _recording_pipeline(params, output_path)
    command = _command_prefix(params) + ["gst-launch-1.0", "-e"] + pipeline

    rospy.loginfo("Recording fisheye video to %s", output_path)
    rospy.loginfo("GStreamer command: %s", " ".join(command))

    process = subprocess.Popen(command, start_new_session=True)
    rospy.on_shutdown(lambda: _terminate(process, params["shutdown_timeout"]))

    try:
        while not rospy.is_shutdown():
            status = process.poll()
            if status is not None:
                if status == 0:
                    rospy.loginfo("GStreamer recorder exited normally")
                else:
                    rospy.logerr("GStreamer recorder exited with status %d", status)
                return
            time.sleep(0.5)
    finally:
        _terminate(process, params["shutdown_timeout"])
        rospy.loginfo("Closed fisheye recording: %s", output_path)


if __name__ == "__main__":
    main()
