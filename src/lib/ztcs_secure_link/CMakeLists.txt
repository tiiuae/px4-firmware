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


px4_add_library(ztcs_secure_link
	secure_link.c
	secure_link_keys.cpp
	identity_px4crypto.cpp
	noise/backend_px4.cpp
	noise/noise_ik.c
	noise/chacha20_ietf.c
	noise/backend_nuttx.c
)

# The Noise core is portable C and the backend is the only part that knows
# the platform. backend_nuttx.c takes SHA-256, HMAC, Poly1305 and X25519 from
# the OS, which needs CONFIG_CRYPTO and CONFIG_CRYPTO_RANDOM_POOL in the NuttX
# configuration: the latter is what builds curve25519.c. A board without them
# supplies its own backend against noise/noise_backend.h.
target_include_directories(ztcs_secure_link PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# The archive, not the nuttx_crypto target: in a kernel build that target
# carries nuttx_kc behind it, and this library is linked into user binaries,
# where the kernel libc pulls in the kernel heap and does not resolve. The
# primitives themselves are ordinary computation and link either side.
if("${PX4_PLATFORM}" MATCHES "nuttx")
	target_link_libraries(ztcs_secure_link PRIVATE $<TARGET_FILE:nuttx_crypto>)
endif()

# The identity key is the one that must be unreadable, and which curve that
# forces depends on the part: identity_px4crypto.cpp signs Ed25519 in an
# on-die enclave, and a board behind a secure element supplies a P-256
# provider against secure_link_identity.h instead.

# The private keys live in the keystore, which this address space cannot
# read, so the static key is a slot index the kernel resolves.
if(PX4_CRYPTO)
	target_compile_definitions(ztcs_secure_link PUBLIC NOISE_STATIC_KEY_BY_INDEX)
	target_link_libraries(ztcs_secure_link PRIVATE crypto_backend_interface)
endif()
