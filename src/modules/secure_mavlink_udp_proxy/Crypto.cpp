#include "Crypto.hpp"

/// Sodium library

bool SodiumCrypto::open(crypto_t type) {
  if (type != crypto_t::CRYPTO_AES) {
    return false;
  }
  randombytes(nonce_, sizeof nonce_);
  return true;
}

bool SodiumCrypto::generate_key(uint8_t idx, bool persistent) {
  randombytes(key_table_[idx], sizeof key_table_[idx]);
  randombytes(nonce_, sizeof nonce_);
  return true;
}

bool SodiumCrypto::encrypt_data(const uint8_t key_index, const uint8_t *message,
                                const size_t message_size, uint8_t *cipher,
                                size_t *cipher_size, uint8_t *mac,
                                size_t *mac_size) {
  unsigned long long sodium_cipher_size = message_size + *mac_size;
  uint8_t *sodium_cipher{static_cast<uint8_t *>(malloc(sodium_cipher_size))};

  int res{crypto_aead_aes256gcm_encrypt(sodium_cipher, &sodium_cipher_size,
                                        message, message_size, NULL, 0, NULL,
                                        nonce_, key_table_[key_index])};
  *cipher_size = message_size;
  *mac_size = static_cast<size_t>(sodium_cipher_size) - message_size;
  memcpy(cipher, sodium_cipher, message_size);
  memcpy(mac, sodium_cipher + *cipher_size, *mac_size);
  free(sodium_cipher);
  return res == 0;
}

bool SodiumCrypto::decrypt_data(const uint8_t key_index, const uint8_t *cipher,
                                const size_t cipher_size, const uint8_t *mac,
                                const size_t mac_size, uint8_t *message,
                                size_t *message_size) {
  unsigned long long mlen{static_cast<unsigned long long>(*message_size)};

  size_t sodium_cipher_size{cipher_size + mac_size};
  uint8_t *sodium_cipher{static_cast<uint8_t *>(malloc(sodium_cipher_size))};
  memcpy(sodium_cipher, cipher, cipher_size);
  memcpy(sodium_cipher + cipher_size, mac, mac_size);
  int res{crypto_aead_aes256gcm_decrypt(message, &mlen, NULL, sodium_cipher,
                                        sodium_cipher_size, NULL, 0, nonce_,
                                        key_table_[key_index])};
  *message_size = static_cast<size_t>(mlen);
  free(sodium_cipher);
  return res == 0;
}

bool SodiumCrypto::renew_nonce(const uint8_t *nonce, size_t nonce_size) {
  if (nonce) {
    memcpy(nonce_, nonce, nonce_size);
  } else {
    randombytes(nonce_, sizeof nonce_);
  }
  return true;
}

bool SodiumCrypto::get_nonce(uint8_t *nonce, size_t *nonce_len) {
  if (nonce) {
    memcpy(nonce, nonce_, sizeof nonce_);
  }
  *nonce_len = sizeof(nonce_);
  return true;
}

/// Crypto++ library
CryptoppCrypto::CryptoppCrypto(const char *filename) {
  CryptoPP::FileSource file(filename, true);
  publicKey_.Load(file);
}

bool CryptoppCrypto::open(const crypto_t type) {
  type_ = type;
  if (!(type == crypto_t::CRYPTO_AES || type == crypto_t::CRYPTO_RSA_OAEP)) {
    fprintf(stderr, "Type: %d\n", type);
    return false;
  }
  prng_.GenerateBlock(nonce_, sizeof nonce_);
  return true;
}

bool CryptoppCrypto::generate_key(const uint8_t idx, const bool persistent) {
  prng_.GenerateBlock(key_table_[idx], key_size_ /*sizeof key_table_[idx]*/);
  prng_.GenerateBlock(nonce_, sizeof nonce_);
  return true;
}

bool CryptoppCrypto::get_encrypted_key(const uint8_t key_idx, uint8_t *key,
                                       size_t *key_len,
                                       const uint8_t encryption_key_idx) {

  // Use RSA key public key to encrypt AES-GCM session key
  CryptoPP::RSAES_OAEP_SHA256_Encryptor rsaEncryptor(publicKey_);
  size_t cipherlen{rsaEncryptor.CiphertextLength(key_size_)};
  if (cipherlen <= *key_len) {
    *key_len = cipherlen;
  } else {
    return false;
  }
  CryptoPP::ArraySource(
      key_table_[key_idx], key_size_, true,
      new CryptoPP::PK_EncryptorFilter(prng_, rsaEncryptor,
                                       new CryptoPP::ArraySink(key, *key_len)));
  return true;
}

bool CryptoppCrypto::signature_check(const uint8_t key_index,
                                     const uint8_t *signature,
                                     const uint8_t *message,
                                     const size_t message_size) {
  CryptoPP::RSASSA_PKCS1v15_SHA256_Verifier verifier(publicKey_);
  return verifier.VerifyMessage(message, message_size, signature, 256);
}

bool CryptoppCrypto::encrypt_data(const uint8_t key_index,
                                  const uint8_t *message,
                                  const size_t message_size, uint8_t *cipher,
                                  size_t *cipher_size, uint8_t *mac,
                                  size_t *mac_size) {
  // print_key(key_index);
  // print_nonce();

  switch (type_) {
  case crypto_t::CRYPTO_AES: {
    CryptoPP::GCM<CryptoPP::AES>::Encryption encryptor;
    encryptor.SetKeyWithIV(key_table_[key_index], key_size_, nonce_,
                           sizeof nonce_);

    if (*mac_size < encryptor.TagSize()) {
      return false;
    }

    *mac_size = encryptor.TagSize();
    *cipher_size = message_size;
    size_t pp_cipher_size{message_size + encryptor.TagSize()};
    *mac_size = encryptor.TagSize();
    uint8_t *pp_cipher{static_cast<uint8_t *>(malloc(pp_cipher_size))};

    try {
      CryptoPP::ArraySource(
          message, message_size, true,
          new CryptoPP::AuthenticatedEncryptionFilter(
              encryptor, new CryptoPP::ArraySink(pp_cipher, pp_cipher_size),
              false, encryptor.TagSize()));
    } catch (const CryptoPP::Exception &e) {
      fprintf(stderr, "Encryption error: %s\n", e.what());
      free(pp_cipher);
      return false;
    }

    memcpy(cipher, pp_cipher, message_size);
    memcpy(mac, pp_cipher + message_size, *mac_size);
    free(pp_cipher);
    return true;
  } break;

  case crypto_t::CRYPTO_RSA_OAEP: {
    // Use RSA key public key to encrypt AES-GCM session key
    CryptoPP::RSAES_OAEP_SHA256_Encryptor rsaEncryptor(publicKey_);

    size_t cipherlen{rsaEncryptor.CiphertextLength(message_size)};
    if (cipherlen <= *cipher_size) {
      *cipher_size = cipherlen;
    } else {
      return false;
    }
    CryptoPP::ArraySource(message, message_size, true,
                          new CryptoPP::PK_EncryptorFilter(
                              prng_, rsaEncryptor,
                              new CryptoPP::ArraySink(cipher, *cipher_size)));
    return true;
  } break;

  default:
    return false;
    break;
  }
}

bool CryptoppCrypto::decrypt_data(const uint8_t key_index,
                                  const uint8_t *cipher,
                                  const size_t cipher_size, const uint8_t *mac,
                                  const size_t mac_size, uint8_t *message,
                                  size_t *message_size) {
  switch (type_) {
  case crypto_t::CRYPTO_AES: {
    CryptoPP::GCM<CryptoPP::AES>::Decryption decryptor;
    decryptor.SetKeyWithIV(key_table_[key_index], key_size_, nonce_,
                           sizeof nonce_);
    *message_size = cipher_size;
    size_t pp_cipher_size{cipher_size + mac_size};
    uint8_t *pp_cipher{static_cast<uint8_t *>(malloc(pp_cipher_size))};
    // MAC AT END:
    memcpy(pp_cipher, cipher, cipher_size);
    memcpy(pp_cipher + cipher_size, mac, mac_size);

    try {
      CryptoPP::ArraySource(
          cipher, cipher_size, true,
          new CryptoPP::AuthenticatedDecryptionFilter(
              decryptor, new CryptoPP::ArraySink(message, *message_size),
              CryptoPP::AuthenticatedDecryptionFilter::MAC_AT_END));
    } catch (const CryptoPP::Exception &e) {
      fprintf(stderr, "Decryption error: %s\n", e.what());
      free(pp_cipher);
      return false;
    }

    free(pp_cipher);
    return true;
  } break;

  default:
    return false;
    break;
  }
}

bool CryptoppCrypto::renew_nonce(const uint8_t *nonce, size_t nonce_size) {
  if (nonce) {
    if (nonce_size != sizeof nonce_) {
      return false;
    }
    memcpy(nonce_, nonce, nonce_size);
  } else {
    prng_.GenerateBlock(nonce_, sizeof nonce_);
  }
  return true;
}

bool CryptoppCrypto::get_nonce(uint8_t *nonce, size_t *nonce_len) {
  if (nonce) {
    memcpy(nonce, nonce_, sizeof nonce_);
  }
  *nonce_len = sizeof(nonce_);
  return true;
}

void CryptoppCrypto::print_nonce() {
  char outstr[200];
  for (size_t i{0}; i < sizeof nonce_; i++) {
    sprintf(outstr + 3 * i, "%02X ", nonce_[i]);
  }
  printf("Nonce (len = %ld): %s\n", sizeof nonce_, outstr);
}

void CryptoppCrypto::print_key(const uint8_t key) {
  char outstr[200];
  for (size_t i{0}; i < key_size_; i++) {
    sprintf(outstr + 3 * i, "%02X ", key_table_[key][i]);
  }
  printf("Key %u (len = %ld): %s\n", key, key_size_, outstr);
}
