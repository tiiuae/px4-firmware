#! /bin/bash -eu

die () {
  echo >&2 "$@"
  exit 1
}

[[ "$#" -eq 3 ]] || die "$0 (operation) (dir) (file)"
DIR=$2
FILE=$3

if [[ -e ${FILE} ]]; then
  echo "${FILE} appeared"
  matching_px4_file=$(echo ${FILE} | sed 's/-updates.txt$/.px4/')
  echo "matching px4 file should be: $matching_px4_file"
  if [[ -e ${matching_px4_file} ]]; then
    echo "updates.txt and px4 file found"

    echo "contents of ${FILE}:"
    cat ${FILE}

    echo "update has finished, exiting"
    # exit with status 1 do signal exit has finished
    exit 1
  fi
fi
