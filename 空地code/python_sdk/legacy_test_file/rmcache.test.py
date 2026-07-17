import pathlib
import sys


def del_pycache(path):
    for p in pathlib.Path(path).rglob("__pycache__"):
        if p.is_dir() and p.name == "__pycache__":
            for pp in p.rglob("*.*"):
                pp.unlink()
        p.rmdir()


del_pycache(".")

# 遍历Python解释器的搜索路径，找到所有包含site-packages的目录
site_packages_path = []
for path in sys.path:
    if path.endswith("site-packages"):
        site_packages_path.append(path)
print(f"site-packages: {site_packages_path}")

# for path in site_packages_path:
#     del_pycache(path)
