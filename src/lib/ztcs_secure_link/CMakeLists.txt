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

if("${PX4_PLATFORM}" MATCHES "nuttx")
	target_link_libraries(ztcs_secure_link PRIVATE nuttx_crypto)
endif()
