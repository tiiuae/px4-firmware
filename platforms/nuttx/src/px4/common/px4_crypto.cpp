/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#if defined(PX4_CRYPTO)

#include <px4_platform_common/crypto.h>
#include <px4_platform_common/crypto_backend.h>
#include <px4_platform_common/defines.h>
#include <px4_platform/board_ctrl.h>

#include <signal.h>
#include <string.h>
#include <unistd.h>

#if defined(PX4_NOISE_KERNEL)
#include <noise_ik.h>
#endif

extern "C" {
#include <nuttx/random.h>
}


px4_sem_t PX4Crypto::_lock;
bool PX4Crypto::_initialized = false;

void PX4Crypto::px4_crypto_init()
{
	if (PX4Crypto::_initialized) {
		return;
	}

	px4_mutex_init(&PX4Crypto::_lock, 0);

	// Initialize nuttx random pool, if it is being used by crypto
#ifdef CONFIG_CRYPTO_RANDOM_POOL
	up_randompool_initialize();
#endif

	// initialize keystore functionality
	keystore_init();

	// initialize actual crypto algoritms
	crypto_init();

	// initialize user ioctl interface for crypto
#if !defined(CONFIG_BUILD_FLAT)
	px4_register_boardct_ioctl(_CRYPTOIOCBASE, crypto_ioctl);
#endif

	PX4Crypto::_initialized = true;
}

PX4Crypto::PX4Crypto()
{
	// Initialize an empty handle
	crypto_session_handle_init(&_crypto_handle);
}

PX4Crypto::~PX4Crypto()
{
	close();
}

bool PX4Crypto::open(px4_crypto_algorithm_t algorithm)
{
	bool ret = false;
	lock();

	// HW specific crypto already open? Just close before proceeding
	if (crypto_session_handle_valid(_crypto_handle)) {
		crypto_close(&_crypto_handle);
	}

	// Open the HW specific crypto handle
	_crypto_handle = crypto_open(algorithm);

	if (crypto_session_handle_valid(_crypto_handle)) {
		ret = true;
	}

	unlock();

	return ret;
}

void PX4Crypto::close()
{
	if (!crypto_session_handle_valid(_crypto_handle)) {
		return;
	}

	lock();
	crypto_close(&_crypto_handle);
	unlock();
}

bool PX4Crypto::sign(uint8_t key_index,
		     uint8_t *signature,
		     const uint8_t *message,
		     size_t message_size)
{
	return crypto_signature_gen(_crypto_handle, key_index, signature, message, message_size);
}

bool PX4Crypto::signature_check(uint8_t key_index,
				const uint8_t *signature,
				const uint8_t *message,
				size_t message_size)
{
	return crypto_signature_check(_crypto_handle, key_index, signature, message, message_size);
}

bool PX4Crypto::get_public_key(uint8_t key_index, uint8_t *pubkey, size_t *pubkey_size)
{
	return crypto_get_public_key(_crypto_handle, key_index, pubkey, pubkey_size);
}

bool PX4Crypto::key_agreement(uint8_t key_index,
			    const uint8_t *peer,
			    size_t peer_size,
			    uint8_t *secret,
			    size_t *secret_size)
{
	return crypto_key_agreement(_crypto_handle, key_index, peer, peer_size, secret, secret_size);
}

bool PX4Crypto::encrypt_data(uint8_t key_index,
			     const uint8_t *message,
			     size_t message_size,
			     uint8_t *cipher,
			     size_t *cipher_size,
			     uint8_t *mac,
			     size_t *mac_size)
{
	return crypto_encrypt_data(_crypto_handle, key_index, message, message_size, cipher, cipher_size, mac, mac_size);
}

bool PX4Crypto::decrypt_data(uint8_t key_index,
			     const uint8_t *cipher,
			     size_t cipher_size,
			     const uint8_t *mac,
			     size_t mac_size,
			     uint8_t *message,
			     size_t *message_size)
{
	return crypto_decrypt_data(_crypto_handle, key_index, cipher, cipher_size, mac, mac_size, message,
				   message_size);
}

bool PX4Crypto::generate_key(uint8_t idx,
			     bool persistent)
{
	return crypto_generate_key(_crypto_handle, idx, persistent);
}

bool PX4Crypto::generate_keypair(size_t key_size,
				 uint8_t key_idx,
				 bool persistent)
{
	return crypto_generate_keypair(_crypto_handle, key_size, key_idx, persistent);
}

bool PX4Crypto::renew_nonce(const uint8_t *nonce,
			    size_t nonce_size)
{
	return crypto_renew_nonce(_crypto_handle, nonce, nonce_size);
}

bool PX4Crypto::get_nonce(uint8_t *nonce,
			  size_t *nonce_len)
{
	return crypto_get_nonce(_crypto_handle, nonce, nonce_len);
}

bool PX4Crypto::set_key(uint8_t authentication_key_idx,
			const uint8_t *signature,
			const uint8_t *key,
			size_t key_len,
			uint8_t key_idx)
{
	return crypto_set_key(_crypto_handle, authentication_key_idx, signature, key, key_len, key_idx);
}

bool PX4Crypto::get_encrypted_key(uint8_t key_idx,
				  uint8_t *key,
				  size_t *key_len,
				  uint8_t encryption_key_idx)
{
	return crypto_get_encrypted_key(_crypto_handle, key_idx, key, key_len, encryption_key_idx);
}

size_t PX4Crypto::get_min_blocksize(uint8_t key_idx)
{
	return crypto_get_min_blocksize(_crypto_handle, key_idx);
}

#if !defined(CONFIG_BUILD_FLAT)
static constexpr int CRYPTO_SESSIONS = 32;
static constexpr size_t CRYPTO_SIGNATURE_MAX = 512;

static struct {
	crypto_session_handle_t handle;
	pid_t owner;
} g_sessions[CRYPTO_SESSIONS];

static px4_sem_t g_sessions_lock = SEM_INITIALIZER(1);

static bool size_in(size_t *size, const size_t *user)
{
	if (user == nullptr || !px4_user_ok(user, sizeof(*user))) {
		return false;
	}

	*size = *user;
	return true;
}

static crypto_session_handle_t *session(const crypto_session_handle_t *user)
{
	crypto_session_handle_t handle;

	if (user == nullptr || !px4_user_ok(user, sizeof(handle))) {
		return nullptr;
	}

	memcpy(&handle, user, sizeof(handle));
	const int i = handle.handle - 1;

	if (i < 0 || i >= CRYPTO_SESSIONS || g_sessions[i].owner != getpid()
	    || !crypto_session_handle_valid(g_sessions[i].handle)) {
		return nullptr;
	}

	return &g_sessions[i].handle;
}

static int session_open(px4_crypto_algorithm_t algorithm)
{
	for (int i = 0; i < CRYPTO_SESSIONS; i++) {
		if (crypto_session_handle_valid(g_sessions[i].handle) && kill(g_sessions[i].owner, 0) < 0) {
			crypto_close(&g_sessions[i].handle);
		}

		if (!crypto_session_handle_valid(g_sessions[i].handle)) {
			g_sessions[i].handle = crypto_open(algorithm);
			g_sessions[i].owner = getpid();
			return crypto_session_handle_valid(g_sessions[i].handle) ? i : -1;
		}
	}

	return -1;
}

#if defined(PX4_NOISE_KERNEL)
static constexpr int NOISE_HANDSHAKES = 6;

static struct {
	struct noise_initiator ini;
	struct noise_static_key link;
	pid_t owner;
} g_handshakes[NOISE_HANDSHAKES];

static void handshake_release(int i)
{
	noise_wipe(&g_handshakes[i], sizeof(g_handshakes[i]));
}

static int handshake_claim()
{
	for (int i = 0; i < NOISE_HANDSHAKES; i++) {
		if (g_handshakes[i].owner != 0 && kill(g_handshakes[i].owner, 0) < 0) {
			handshake_release(i);
		}

		if (g_handshakes[i].owner == 0) {
			g_handshakes[i].owner = getpid();
			return i;
		}
	}

	return -1;
}

static int handshake(int handle)
{
	const int i = handle - 1;

	if (i < 0 || i >= NOISE_HANDSHAKES || g_handshakes[i].owner == 0 || g_handshakes[i].owner != getpid()) {
		return -1;
	}

	return i;
}
#endif

static int crypto_ioctl_locked(unsigned int cmd, unsigned long arg)
{
	crypto_session_handle_t *s = nullptr;
	size_t n = 0;
	size_t m = 0;

	switch (cmd) {
	case CRYPTOIOCOPEN: {
			px4_user_arg<cryptoiocopen_t> d;

			if (!d.in(arg) || d->handle == nullptr || !px4_user_ok(d->handle, sizeof(*d->handle))) {
				return -EFAULT;
			}

			crypto_session_handle_t user{};
			const int i = session_open(d->algorithm);

			if (i >= 0) {
				user = g_sessions[i].handle;
				user.handle = i + 1;
				user.context = nullptr;
			}

			memcpy(d->handle, &user, sizeof(user));
			return PX4_OK;
		}

	case CRYPTOIOCCLOSE:
		if ((s = session((const crypto_session_handle_t *)arg)) == nullptr) {
			return -EFAULT;
		}

		crypto_close(s);
		((crypto_session_handle_t *)arg)->handle = 0;
		return PX4_OK;

	case CRYPTOIOCENCRYPT: {
			px4_user_arg<cryptoiocencrypt_t> d;

			if (!d.in(arg) || (s = session(d->handle)) == nullptr || !size_in(&n, d->cipher_size)
			    || (d->mac_size != nullptr && !size_in(&m, d->mac_size)) || !px4_user_ok(d->message, d->message_size)
			    || !px4_user_ok(d->cipher, n) || !px4_user_ok(d->mac, m)) {
				return -EFAULT;
			}

			const bool ret = crypto_encrypt_data(*s, d->key_index, d->message, d->message_size, d->cipher, &n, d->mac,
							     d->mac_size != nullptr ? &m : nullptr);
			*d->cipher_size = n;

			if (d->mac_size != nullptr) {
				*d->mac_size = m;
			}

			((cryptoiocencrypt_t *)arg)->ret = ret;
			return PX4_OK;
		}

	case CRYPTOIOCGENKEY: {
			px4_user_arg<cryptoiocgenkey_t> d;

			if (!d.in(arg) || (s = session(d->handle)) == nullptr) {
				return -EFAULT;
			}

			((cryptoiocgenkey_t *)arg)->ret = crypto_generate_key(*s, d->idx, d->persistent);
			return PX4_OK;
		}

	case CRYPTOIOCGENKEYPAIR: {
			px4_user_arg<cryptoiocgenkeypair_t> d;

			if (!d.in(arg) || (s = session(d->handle)) == nullptr) {
				return -EFAULT;
			}

			((cryptoiocgenkeypair_t *)arg)->ret = crypto_generate_keypair(*s, d->key_size, d->key_idx, d->persistent);
			return PX4_OK;
		}

	case CRYPTOIOCRENEWNONCE: {
			px4_user_arg<cryptoiocrenewnonce_t> d;

			if (!d.in(arg) || (s = session(d->handle)) == nullptr || !px4_user_ok(d->nonce, d->nonce_size)) {
				return -EFAULT;
			}

			((cryptoiocrenewnonce_t *)arg)->ret = crypto_renew_nonce(*s, d->nonce, d->nonce_size);
			return PX4_OK;
		}

	case CRYPTOIOCGETNONCE: {
			px4_user_arg<cryptoiocgetnonce_t> d;

			if (!d.in(arg) || (s = session(d->handle)) == nullptr || !size_in(&n, d->nonce_len)
			    || !px4_user_ok(d->nonce, n)) {
				return -EFAULT;
			}

			const bool ret = crypto_get_nonce(*s, d->nonce, &n);
			*d->nonce_len = n;
			((cryptoiocgetnonce_t *)arg)->ret = ret;
			return PX4_OK;
		}

	case CRYPTOIOCSETKEY: {
			px4_user_arg<cryptoiocsetkey_t> d;

			if (!d.in(arg) || (s = session(d->handle)) == nullptr || !px4_user_ok(d->signature, CRYPTO_SIGNATURE_MAX)
			    || !px4_user_ok(d->key, d->key_len)) {
				return -EFAULT;
			}

			((cryptoiocsetkey_t *)arg)->ret = crypto_set_key(*s, d->authentication_key_idx, d->signature, d->key, d->key_len,
							  d->key_idx);
			return PX4_OK;
		}

	case CRYPTOIOCGETKEY: {
			px4_user_arg<cryptoiocgetkey_t> d;

			if (!d.in(arg) || (s = session(d->handle)) == nullptr || !size_in(&n, d->max_len)
			    || !px4_user_ok(d->key, n)) {
				return -EFAULT;
			}

			const bool ret = crypto_get_encrypted_key(*s, d->key_idx, d->key, &n, d->encryption_key_idx);
			*d->max_len = n;
			((cryptoiocgetkey_t *)arg)->ret = ret;
			return PX4_OK;
		}

	case CRYPTOIOCSIGN: {
			px4_user_arg<cryptoiocsign_t> d;

			if (!d.in(arg) || (s = session(d->handle)) == nullptr || !px4_user_ok(d->signature, CRYPTO_SIGNATURE_MAX)
			    || !px4_user_ok(d->message, d->message_size)) {
				return -EFAULT;
			}

			((cryptoiocsign_t *)arg)->ret = crypto_signature_gen(*s, d->key_index, d->signature, d->message, d->message_size);
			return PX4_OK;
		}

	case CRYPTOIOCGETPUBLICKEY: {
			px4_user_arg<cryptoiocgetpublickey_t> d;

			if (!d.in(arg) || (s = session(d->handle)) == nullptr || !size_in(&n, d->pubkey_size)
			    || !px4_user_ok(d->pubkey, n)) {
				return -EFAULT;
			}

			const bool ret = crypto_get_public_key(*s, d->key_index, d->pubkey, &n);
			*d->pubkey_size = n;
			((cryptoiocgetpublickey_t *)arg)->ret = ret;
			return PX4_OK;
		}

	case CRYPTOIOCKEYAGREEMENT: {
			px4_user_arg<cryptoiockeyagreement_t> d;

			if (!d.in(arg) || (s = session(d->handle)) == nullptr || !px4_user_ok(d->peer, d->peer_size)
			    || !size_in(&n, d->secret_size) || !px4_user_ok(d->secret, n)) {
				return -EFAULT;
			}

			const bool ret = crypto_key_agreement(*s, d->key_index, d->peer, d->peer_size, d->secret, &n);
			*d->secret_size = n;
			((cryptoiockeyagreement_t *)arg)->ret = ret;
			return PX4_OK;
		}

	case CRYPTOIOCSIGNATURECHECK: {
			px4_user_arg<cryptoiocsignaturecheck_t> d;

			if (!d.in(arg) || (s = session(d->handle)) == nullptr || !px4_user_ok(d->signature, CRYPTO_SIGNATURE_MAX)
			    || !px4_user_ok(d->message, d->message_size)) {
				return -EFAULT;
			}

			((cryptoiocsignaturecheck_t *)arg)->ret = crypto_signature_check(*s, d->key_index, d->signature, d->message,
					d->message_size);
			return PX4_OK;
		}

	case CRYPTOIOCGETBLOCKSZ: {
			px4_user_arg<cryptoiocgetblocksz_t> d;

			if (!d.in(arg) || (s = session(d->handle)) == nullptr) {
				return -EFAULT;
			}

			((cryptoiocgetblocksz_t *)arg)->ret = crypto_get_min_blocksize(*s, d->key_idx);
			return PX4_OK;
		}

	case CRYPTOIOCDECRYPTDATA: {
			px4_user_arg<cryptoiocdecryptdata_t> d;

			if (!d.in(arg) || (s = session(d->handle)) == nullptr || !px4_user_ok(d->cipher, d->cipher_size)
			    || !px4_user_ok(d->mac, d->mac_size) || !size_in(&n, d->message_size) || !px4_user_ok(d->message, n)) {
				return -EFAULT;
			}

			const bool ret = crypto_decrypt_data(*s, d->key_index, d->cipher, d->cipher_size, d->mac, d->mac_size, d->message, &n);
			*d->message_size = n;
			((cryptoiocdecryptdata_t *)arg)->ret = ret;
			return PX4_OK;
		}

#if defined(PX4_NOISE_KERNEL)

	case CRYPTOIOCNOISESTART: {
			px4_user_arg<cryptoiocnoisestart_t> d;
			uint8_t rs[NOISE_DHLEN];
			uint8_t identity[NOISE_IDENTITY_PAYLOAD_LEN];
			uint8_t msg[NOISE_MSG1_LEN];
			size_t len = sizeof(msg);

			if (!d.in(arg) || d->remote_static == nullptr || !px4_user_ok(d->remote_static, sizeof(rs))
			    || d->identity == nullptr || d->identity_size != sizeof(identity) || !px4_user_ok(d->identity, sizeof(identity))
			    || !size_in(&m, d->message_size) || m < sizeof(msg)
			    || d->message == nullptr || !px4_user_ok(d->message, sizeof(msg))) {
				return -EFAULT;
			}

			memcpy(rs, d->remote_static, sizeof(rs));
			memcpy(identity, d->identity, sizeof(identity));

			const int i = handshake_claim();
			int rc = NOISE_ERR_BACKEND;

			if (i >= 0) {
				g_handshakes[i].link.index = d->link_index;
				rc = noise_initiator_start(&g_handshakes[i].ini, &g_handshakes[i].link, rs, identity, msg, &len);

				if (rc != NOISE_OK) {
					handshake_release(i);
				}
			}

			if (rc == NOISE_OK) {
				memcpy(d->message, msg, len);
				*d->message_size = len;
			}

			noise_wipe(msg, sizeof(msg));
			((cryptoiocnoisestart_t *)arg)->handle = rc == NOISE_OK ? i + 1 : rc;
			return PX4_OK;
		}

	case CRYPTOIOCNOISEFINISH: {
			px4_user_arg<cryptoiocnoisefinish_t> d;
			uint8_t msg[NOISE_MSG2_LEN];
			struct noise_session session;

			if (!d.in(arg) || d->message == nullptr || !px4_user_ok(d->message, d->message_size)
			    || d->send_index == nullptr || !px4_user_ok(d->send_index, 1)
			    || d->recv_index == nullptr || !px4_user_ok(d->recv_index, 1)) {
				return -EFAULT;
			}

			const int i = handshake(d->handle);
			int rc = i < 0 ? NOISE_ERR_STATE : NOISE_ERR_INPUT;

			if (i >= 0 && d->message_size == sizeof(msg)) {
				memcpy(msg, d->message, sizeof(msg));
				rc = noise_initiator_finish(&g_handshakes[i].ini, msg, sizeof(msg), &session);
			}

			if (rc == NOISE_OK) {
				*d->send_index = session.send.index;
				*d->recv_index = session.recv.index;
				handshake_release(i);
			}

			((cryptoiocnoisefinish_t *)arg)->ret = rc;
			return PX4_OK;
		}

	case CRYPTOIOCNOISEABORT: {
			const int i = handshake((int)arg);

			if (i >= 0) {
				handshake_release(i);
			}

			return PX4_OK;
		}

#endif

	default:
		return PX4_ERROR;
	}
}

int PX4Crypto::crypto_ioctl(unsigned int cmd, unsigned long arg)
{
	px4_sem_wait(&g_sessions_lock);
	const int ret = crypto_ioctl_locked(cmd, arg);
	px4_sem_post(&g_sessions_lock);
	return ret;
}
#endif // !defined(CONFIG_BUILD_FLAT)

#endif
