#!/bin/bash

if [ "$#" -eq 1 ]; then
    username="$1"
else
    username=$(whoami)
fi

if [ -z "$username" ]; then
    echo "Error: Username not provided."
    exit 1
fi

root_directory="./src"

new_shebang_line="#!/home/$username/parrot/bin/python"

find "$root_directory" -type f -name "*.py" -print0 | while IFS= read -r -d '' file; do
    # Open each file and replace the first line
    sed -i '1s|^.*$|'"$new_shebang_line"'|' "$file"
    echo "Updated shebang line in: $file"
done