############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################


px4_add_library(secure_mavlink
	secure_link.c
	cobs.c
	secure_link_keys.cpp
	identity_px4crypto.cpp
	noise/backend_px4.cpp
	noise/sm-noise-c/noise_ik.c
	noise/sm-noise-c/chacha20_ietf.c
	noise/sm-noise-c/backend_nuttx.c
)

# The Noise core is portable C and the backend is the only part that knows
# the platform. backend_nuttx.c takes SHA-256, HMAC, Poly1305 and X25519 from
# the OS, which needs CONFIG_CRYPTO and CONFIG_CRYPTO_RANDOM_POOL in the NuttX
# configuration: the latter is what builds curve25519.c. A board without them
# supplies its own backend against noise/sm-noise-c/noise_backend.h.
target_include_directories(secure_mavlink PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/noise/sm-noise-c
)

if(PX4_CRYPTO AND NOT DEFINED CONFIG_BUILD_FLAT AND "${PX4_PLATFORM}" MATCHES "nuttx")
	target_sources(secure_mavlink PRIVATE handshake_kernel.cpp)
	target_compile_definitions(secure_mavlink PUBLIC NOISE_HANDSHAKE_IN_KERNEL)
else()
	target_sources(secure_mavlink PRIVATE handshake_local.c)
endif()

# The archive, not the nuttx_crypto target: in a kernel build that target
# carries nuttx_kc behind it, and this library is linked into user binaries,
# where the kernel libc pulls in the kernel heap and does not resolve. The
# primitives themselves are ordinary computation and link either side.
if("${PX4_PLATFORM}" MATCHES "nuttx")
	if(CONFIG_BUILD_FLAT)
		target_link_libraries(secure_mavlink PRIVATE nuttx_crypto)
	else()
		target_link_libraries(secure_mavlink PRIVATE $<TARGET_FILE:nuttx_crypto>)
	endif()
endif()

# The identity key is the one that must be unreadable: identity_px4crypto.cpp
# signs P-256 in the i.MX9 enclave or Ed25519 in the keystore, and a board
# behind a secure element supplies its own provider against
# secure_link_identity.h.

# The private keys live in the keystore and the transport keys in the kernel's
# key cache, neither readable from this address space, so both are slot
# indices the kernel resolves.
if(PX4_CRYPTO)
	target_compile_definitions(secure_mavlink PUBLIC NOISE_STATIC_KEY_BY_INDEX NOISE_SESSION_KEY_BY_INDEX)
	target_link_libraries(secure_mavlink PRIVATE crypto_backend_interface)
endif()

