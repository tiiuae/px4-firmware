#!/usr/bin/env bash
# Launch the px4-bench MCP server, creating its virtualenv on first run.
# stdout is the MCP protocol channel: everything else must go to stderr.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
venv="${here}/.venv"
stamp="${venv}/.requirements.sha"
req_sha="$(sha256sum "${here}/requirements.txt" | cut -d' ' -f1)"

if [ ! -x "${venv}/bin/python" ]; then
	python3 -m venv "${venv}" >&2
fi
if [ "$(cat "${stamp}" 2>/dev/null)" != "${req_sha}" ]; then
	"${venv}/bin/pip" install -q --upgrade pip >&2
	"${venv}/bin/pip" install -q -r "${here}/requirements.txt" >&2
	echo "${req_sha}" > "${stamp}"
fi

cd "${here}"
exec "${venv}/bin/python" -m px4_bench "$@"
