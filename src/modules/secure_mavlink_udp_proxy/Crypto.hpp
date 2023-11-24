#ifndef PROXY_CRYPTO_HPP_
#define PROXY_CRYPTO_HPP_

#include <cryptopp/files.h>
#include <cryptopp/filters.h>
#include <cryptopp/gcm.h>
#include <cryptopp/osrng.h>
#include <cryptopp/rsa.h>
#include <sodium.h>

typedef enum {
  CRYPTO_NONE = 0,
  CRYPTO_ED25519 = 1,
  CRYPTO_XCHACHA20 = 2,
  CRYPTO_AES = 3,
  CRYPTO_RSA_OAEP = 4,
  CRYPTO_ECDSA_P256 = 5,
  CRYPTO_RSA_SIG = 6,
} crypto_t;

// global keystore for all
extern uint8_t key_table_[256][crypto_aead_aes256gcm_KEYBYTES];

class Crypto {
public:
  virtual bool open(crypto_t type) = 0;
  virtual bool generate_key(uint8_t idx, bool persistent) = 0;
  virtual bool get_encrypted_key(uint8_t key_idx, uint8_t *key, size_t *key_len,
                                 uint8_t encryption_key_idx) = 0;
  virtual bool signature_check(uint8_t key_index, const uint8_t *signature,
                               const uint8_t *message, size_t message_size) = 0;
  virtual bool encrypt_data(uint8_t key_index, const uint8_t *message,
                            size_t message_size, uint8_t *cipher,
                            size_t *cipher_size, uint8_t *mac,
                            size_t *mac_size) = 0;
  virtual bool decrypt_data(uint8_t key_index, const uint8_t *cipher,
                            size_t cipher_size, const uint8_t *mac,
                            size_t mac_size, uint8_t *message,
                            size_t *message_size) = 0;
  virtual bool renew_nonce(const uint8_t *nonce, size_t nonce_size) = 0;
  virtual bool get_nonce(uint8_t *nonce, size_t *nonce_len) = 0;
  virtual void close() = 0;

protected:
  crypto_t type_{crypto_t::CRYPTO_NONE};
  uint8_t nonce_[crypto_aead_aes256gcm_NPUBBYTES];
};

class SodiumCrypto : public Crypto {
public:
  bool open(crypto_t type);
  bool generate_key(uint8_t idx, bool persistent);
  bool get_encrypted_key(uint8_t key_idx, uint8_t *key, size_t *key_len,
                         uint8_t encryption_key_idx) {
    return false;
  }
  bool signature_check(uint8_t key_index, const uint8_t *signature,
                       const uint8_t *message, size_t message_size) {
    return false;
  }
  bool encrypt_data(uint8_t key_index, const uint8_t *message,
                    size_t message_size, uint8_t *cipher, size_t *cipher_size,
                    uint8_t *mac, size_t *mac_size);
  bool decrypt_data(uint8_t key_index, const uint8_t *cipher,
                    size_t cipher_size, const uint8_t *mac, size_t mac_size,
                    uint8_t *message, size_t *message_size);
  bool renew_nonce(const uint8_t *nonce, size_t nonce_size);
  bool get_nonce(uint8_t *nonce, size_t *nonce_len);
  void close() {}
};
static void sodium_crypto_init(void) {
  if (sodium_init() < 0) {
    exit(-1);
  }
}

class CryptoppCrypto : public Crypto {
public:
  CryptoppCrypto(const char *filename = "publickey.der");
  bool open(crypto_t type);
  bool generate_key(uint8_t idx, bool persistent);
  bool get_encrypted_key(uint8_t key_idx, uint8_t *key, size_t *key_len,
                         uint8_t encryption_key_idx);
  bool signature_check(uint8_t key_index, const uint8_t *signature,
                       const uint8_t *message, size_t message_size);
  bool encrypt_data(uint8_t key_index, const uint8_t *message,
                    size_t message_size, uint8_t *cipher, size_t *cipher_size,
                    uint8_t *mac, size_t *mac_size);
  bool decrypt_data(uint8_t key_index, const uint8_t *cipher,
                    size_t cipher_size, const uint8_t *mac, size_t mac_size,
                    uint8_t *message, size_t *message_size);
  bool renew_nonce(const uint8_t *nonce, size_t nonce_size);
  bool get_nonce(uint8_t *nonce, size_t *nonce_len);
  void close() {}
  void print_nonce();
  void print_key(uint8_t key);

private:
  CryptoPP::RSA::PublicKey publicKey_;
  CryptoPP::AutoSeededRandomPool prng_;
  const size_t key_size_ = crypto_aead_aes256gcm_KEYBYTES;
};

#endif // PROXY_CRYPTO_HPP_
