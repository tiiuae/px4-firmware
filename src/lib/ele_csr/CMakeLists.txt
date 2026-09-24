############################################################################
#
#   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
#
############################################################################
px4_add_library(ele_csr
	ele_csr.cpp
)

target_include_directories(ele_csr PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Headers only: this is user space, so the crypto calls cross into the kernel
# through PX4Crypto rather than linking the backend.
# No libtomcrypt: it is kernel side, so the enclave hashes and base64 is local.
target_link_libraries(ele_csr PRIVATE crypto_backend_interface)
