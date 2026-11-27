"""Create an isolated MASt3R pair-inference environment at analysis/local/mast3r."""
import argparse
import hashlib
from pathlib import Path
import os
import subprocess
import sys
import venv


HERE = Path(__file__).resolve().parent
DEFAULT_RUNTIME = HERE.parent / "local" / "mast3r"
SOURCE_URL = "https://github.com/naver/mast3r.git"
SOURCE_COMMIT = "f5209afc300cec36239a7ac992263f36847bbba0"
MODEL_REPO = "naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric"
MODEL_REV = "06e7259f34c3060f322df5cb0c7b9094f57e41fc"
WEIGHT_SHA256 = "0a615eb05fa9db654050aa655945ee5696e7c6c1b7f93f1ee8c37249010f6feb"


def download_weights(destination):
    from huggingface_hub import hf_hub_download
    for name in ("config.json", "model.safetensors"):
        hf_hub_download(MODEL_REPO, name, revision=MODEL_REV, local_dir=destination)
    digest = hashlib.sha256()
    with (destination / "model.safetensors").open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    if digest.hexdigest() != WEIGHT_SHA256:
        raise ValueError("Official model.safetensors SHA-256 mismatch")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, default=DEFAULT_RUNTIME)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    parser.add_argument("--skip-weights", action="store_true", help="Only install source and packages")
    parser.add_argument("--download-only", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    runtime = args.runtime.resolve()
    if args.download_only:
        download_weights(runtime / "model")
        return
    if not ((3, 9) <= sys.version_info[:2] <= (3, 12)):
        parser.error("Use Python 3.9 through 3.12 with the pinned PyTorch wheels")
    runtime.mkdir(parents=True, exist_ok=True)
    source = runtime / "source"
    if not (source / ".git").is_dir():
        subprocess.run(["git", "clone", "--no-checkout", SOURCE_URL, str(source)], check=True)
    subprocess.run(["git", "-C", str(source), "checkout", "--detach", SOURCE_COMMIT], check=True)
    subprocess.run(["git", "-C", str(source), "submodule", "update", "--init", "--recursive"], check=True)
    env = runtime / "venv"
    python = env / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    if not python.exists():
        venv.EnvBuilder(with_pip=True).create(env)
    index = "https://download.pytorch.org/whl/" + ("cu121" if args.device == "cuda" else "cpu")
    subprocess.run([str(python), "-m", "pip", "install", "torch==2.5.1", "torchvision==0.20.1",
                    "--index-url", index], check=True)
    subprocess.run([str(python), "-m", "pip", "install", "-r", str(HERE / "requirements.txt")], check=True)
    if args.device == "cuda":
        subprocess.run([str(python), "-c", "import torch; assert torch.cuda.is_available(), 'CUDA device unavailable'"], check=True)
    if not args.skip_weights:
        subprocess.run([str(python), str(Path(__file__).resolve()), "--runtime", str(runtime), "--download-only"], check=True)
    print("Interpreter:", python)
    print("Source:", source)
    print("Weights:", runtime / "model")


if __name__ == "__main__":
    main()
