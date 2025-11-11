#!/bin/sh -eu

# this shell script validates the ota packages are extracted correctly
# to be used with docker health check
# reads saluki_file_info.json and check all matching validation files exist

SALUKI_FILE_INFO_JSON="/Firmware/saluki_file_info.json"

# find px4 (.bin) files from the saluki_file_info.json
px4_files() {
    jq -r '.files[] | select (.type == "px4-bin") | .filename' <"${SALUKI_FILE_INFO_JSON}"
}

# validate each px4 file has a matching validation file
validate_px4_file() {
    # px4 .bin should have matching .val file
    validation_file="/Firmware/$(basename ${px4_bin_filename%.bin}).val"

    if [ ! -f ${validation_file} ]; then
        echo "Validation file ${validation_file} does not exist for ${px4_bin_filename}!"
        return 1
    fi

    return 0
}

# find px4 files we should have in the system based on saluki_file_info.json
px4_files=$(px4_files)
if [ -z "${px4_files}" ]; then
    echo "No px4 files found in ${SALUKI_FILE_INFO_JSON}"
    exit 1
fi

# validate all expected px4 validation files
for px4_bin_filename in ${px4_files}; do
    if ! validate_px4_file "${px4_bin_filename}"; then
        echo "Validation failed for ${px4_bin_filename}"
        exit 1
    fi
done

echo "PX4 ota update files validated successfully."
