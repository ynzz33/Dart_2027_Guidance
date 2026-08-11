"""
convert_mjpeg.py — 将 OpenMV 录制的 .mjpeg 文件转为带索引的 .mp4。

问题背景:
  OpenMV 的 mjpeg.Mjpeg 模块生成的 .mjpeg 文件是原始 MJPEG 码流，
  没有容器格式的帧索引表 (如 MP4 的 MOOV atom)。
  VLC 打开后会弹出"文件索引损坏或缺失"提示，导致进度条、快进/快退无法使用。

解决方案:
  用 FFmpeg 将 .mjpeg 转码为 .mp4，并通过 -movflags +faststart
  把索引信息写入文件头部，播放器一打开就能读取完整的帧索引。

用法:
  python convert_mjpeg.py                          # 转换当前目录下所有 .mjpeg
  python convert_mjpeg.py record.mjpeg             # 转换单个文件
  python convert_mjpeg.py a.mjpeg b.mjpeg          # 转换多个文件
  python convert_mjpeg.py --keep-original          # 转换后保留原始 .mjpeg
  python convert_mjpeg.py --dir ./recordings       # 转换指定目录下所有 .mjpeg
"""

import subprocess
import os
import sys
import glob
import shutil
from pathlib import Path


def find_ffmpeg():
    """查找 ffmpeg 可执行文件路径"""
    # 先检查系统 PATH
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg:
        return ffmpeg

    # Windows 常见安装路径
    local_appdata = os.environ.get("LOCALAPPDATA", os.path.expanduser("~\\AppData\\Local"))
    common_paths = [
        "C:\\ffmpeg\\bin\\ffmpeg.exe",
        "C:\\Program Files\\ffmpeg\\bin\\ffmpeg.exe",
        os.path.expanduser("~\\ffmpeg\\bin\\ffmpeg.exe"),
        # winget (Gyan.FFmpeg)
        os.path.join(local_appdata, "Microsoft\\WinGet\\Packages\\Gyan.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe\\ffmpeg-*-full_build\\bin\\ffmpeg.exe"),
    ]
    # 展开通配符
    import glob as _glob
    expanded = []
    for p in common_paths:
        if "*" in p:
            matches = sorted(_glob.glob(p), reverse=True)
            expanded.extend(matches)
        else:
            expanded.append(p)
    common_paths = expanded
    for p in common_paths:
        if os.path.isfile(p):
            return p

    return None


def get_file_size_mb(path):
    """获取文件大小 (MB)"""
    return os.path.getsize(path) / (1024 * 1024)


def convert_single(input_path, output_path=None, keep_original=False):
    """
    转换单个 .mjpeg 文件为 .mp4。

    参数:
        input_path:   输入 .mjpeg 文件路径
        output_path:  输出 .mp4 路径 (默认同目录、同名、.mp4 后缀)
        keep_original: 是否保留原始 .mjpeg 文件

    返回:
        True 成功, False 失败
    """
    if not os.path.exists(input_path):
        print(f"  [错误] 文件不存在: {input_path}")
        return False

    if output_path is None:
        output_path = os.path.splitext(input_path)[0] + ".mp4"

    size_mb = get_file_size_mb(input_path)
    print(f"  输入: {input_path} ({size_mb:.1f} MB)")
    print(f"  输出: {output_path}")

    # FFmpeg 命令:
    #   -i 输入文件
    #   -c:v libx264  使用 H.264 编码 (兼容性最好)
    #   -preset fast   编码速度优先
    #   -crf 23        质量控制 (越小越清晰, 18-28 是常用范围)
    #   -movflags +faststart  把索引(moov atom)移到文件头 — 这是解决问题的关键参数
    #   -y             覆盖已存在的输出文件
    cmd = [
        "ffmpeg",
        "-i", input_path,
        "-c:v", "libx264",
        "-preset", "fast",
        "-crf", "23",
        "-movflags", "+faststart",
        "-y",
        output_path,
    ]

    try:
        print(f"  转换中...")
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=3600,  # 大文件可能很久, 给 1 小时
        )

        if result.returncode != 0:
            print(f"  [失败] FFmpeg 返回码: {result.returncode}")
            # 打印最后几行错误信息
            stderr_lines = result.stderr.strip().split("\n")
            for line in stderr_lines[-5:]:
                print(f"         {line}")
            return False

        # 验证输出文件
        if not os.path.exists(output_path) or os.path.getsize(output_path) == 0:
            print(f"  [失败] 输出文件为空或不存在")
            return False

        out_size_mb = get_file_size_mb(output_path)
        ratio = out_size_mb / size_mb * 100 if size_mb > 0 else 0
        print(f"  [完成] 输出大小: {out_size_mb:.1f} MB ({ratio:.0f}%)")

        # 删除原始文件
        if not keep_original:
            os.remove(input_path)
            print(f"  已删除原始文件: {os.path.basename(input_path)}")
        else:
            print(f"  保留原始文件: {os.path.basename(input_path)}")

        return True

    except subprocess.TimeoutExpired:
        print(f"  [失败] 转换超时 (>1小时), 文件可能过大")
        return False
    except Exception as e:
        print(f"  [失败] {e}")
        return False


def main():
    # ============ 参数解析 ============
    keep_original = False
    target_dir = None
    input_files = []

    args = sys.argv[1:]
    i = 0
    while i < len(args):
        arg = args[i]
        if arg in ("-h", "--help"):
            print(__doc__)
            return
        elif arg in ("--keep-original", "-k"):
            keep_original = True
        elif arg in ("--dir", "-d"):
            if i + 1 < len(args):
                target_dir = args[i + 1]
                i += 1
            else:
                print("[错误] --dir 需要指定目录路径")
                sys.exit(1)
        else:
            input_files.append(arg)
        i += 1

    # ============ 收集输入文件 ============
    if target_dir:
        # 指定目录下所有 .mjpeg
        pattern = os.path.join(target_dir, "*.mjpeg")
        input_files = glob.glob(pattern)
        if not input_files:
            print(f"[提示] 目录 {target_dir} 中没有找到 .mjpeg 文件")
            return
    elif not input_files:
        # 默认: 当前目录下所有 .mjpeg
        input_files = glob.glob("*.mjpeg")
        if not input_files:
            print("[提示] 当前目录没有 .mjpeg 文件")
            print("用法: python convert_mjpeg.py <文件.mjpeg> [更多文件...]")
            print("      python convert_mjpeg.py --dir <目录>")
            print("      python convert_mjpeg.py --help")
            return

    # ============ 检查 FFmpeg ============
    ffmpeg_path = find_ffmpeg()
    if not ffmpeg_path:
        print("=" * 55)
        print("  [错误] 未找到 FFmpeg!")
        print("")
        print("  请先安装 FFmpeg:")
        print("    1. 下载: https://ffmpeg.org/download.html")
        print("    2. 将 ffmpeg.exe 所在目录添加到系统 PATH")
        print("    3. 或把 ffmpeg.exe 放在 C:\\ffmpeg\\bin\\ 下")
        print("=" * 55)
        sys.exit(1)

    print(f"FFmpeg: {ffmpeg_path}")
    print(f"共找到 {len(input_files)} 个 .mjpeg 文件")
    if keep_original:
        print("模式: 保留原始文件")
    else:
        print("模式: 转换后删除原始文件")
    print("-" * 50)

    # ============ 批量转换 ============
    success = 0
    fail = 0

    for i, f in enumerate(input_files, 1):
        print(f"\n[{i}/{len(input_files)}]")
        if convert_single(f, keep_original=keep_original):
            success += 1
        else:
            fail += 1

    # ============ 汇总 ============
    print("\n" + "=" * 50)
    print(f"转换完成! 成功: {success}, 失败: {fail}")
    if success > 0:
        print("转换后的 .mp4 文件可以在 VLC 中正常定位、快进/快退了。")
    print("=" * 50)


if __name__ == "__main__":
    main()
