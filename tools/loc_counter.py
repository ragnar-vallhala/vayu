import os
import sys

def count_loc_in_file(filepath):
    loc = 0
    in_block_comment = False

    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                line = line.strip()

                if not line:
                    continue

                if in_block_comment:
                    if "*/" in line:
                        in_block_comment = False
                        line = line.split("*/", 1)[1].strip()
                        if not line:
                            continue
                    else:
                        continue

                if "/*" in line:
                    if "*/" in line:
                        before = line.split("/*", 1)[0].strip()
                        if before:
                            loc += 1
                        continue
                    else:
                        in_block_comment = True
                        before = line.split("/*", 1)[0].strip()
                        if before:
                            loc += 1
                        continue

                if line.startswith("//"):
                    continue

                if "//" in line:
                    line = line.split("//", 1)[0].strip()
                    if not line:
                        continue

                loc += 1

    except Exception as e:
        print(f"Error reading {filepath}: {e}")

    return loc


def scan_directory(root_dir, exclude_dirs):
    total_loc = 0
    file_count = 0
    results = []  # 🔥 store (path, loc)

    for dirpath, dirnames, filenames in os.walk(root_dir):
        dirnames[:] = [d for d in dirnames if d not in exclude_dirs]

        for file in filenames:
            if file.endswith(".c") or file.endswith(".h"):
                full_path = os.path.join(dirpath, file)
                loc = count_loc_in_file(full_path)
                results.append((full_path, loc))
                total_loc += loc
                file_count += 1

    # 🔥 sort by LOC (descending)
    results.sort(key=lambda x: x[1], reverse=True)

    # print sorted
    for path, loc in results:
        print(f"{path}: {loc} LOC")

    print("\n========== SUMMARY ==========")
    print(f"Files scanned : {file_count}")
    print(f"Total LOC     : {total_loc}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python count_loc.py <directory> [exclude_dir1 exclude_dir2 ...]")
        sys.exit(1)

    directory = sys.argv[1]
    exclude_dirs = sys.argv[2:]

    if not os.path.isdir(directory):
        print("Invalid directory!")
        sys.exit(1)

    print(f"Excluding directories: {exclude_dirs}\n")
    scan_directory(directory, exclude_dirs)