#include <unit_test.h>
#include <px4_platform_common/crypto.h>
#include <px4_platform_common/log.h>

class SessionSlotTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool test_foreign_slots();
	bool test_own_slot();

	bool seal(uint8_t idx);

	PX4Crypto _crypto;
	const uint8_t _key[32] {0x5a};
};

bool SessionSlotTest::seal(uint8_t idx)
{
	uint8_t iv[12] {};
	uint8_t text[16] {};
	uint8_t cipher[16];
	uint8_t tag[16];
	size_t cipher_len = sizeof(cipher);
	size_t tag_len = sizeof(tag);

	return _crypto.renew_nonce(iv, sizeof(iv))
	       && _crypto.encrypt_data(idx, text, sizeof(text), cipher, &cipher_len, tag, &tag_len);
}

bool SessionSlotTest::test_foreign_slots()
{
	int held = 0;

	for (uint8_t i = 0; i < CRYPTO_SESSION_KEY_COUNT; i++) {
		const uint8_t idx = CRYPTO_SESSION_KEY_FIRST + i;

		ut_assert("a slot this process never set seals nothing", !seal(idx));

		if (_crypto.set_key(0, nullptr, _key, sizeof(_key), idx)) {
			ut_assert_true(_crypto.set_key(0, nullptr, nullptr, 0, idx));
			continue;
		}

		held++;
		ut_assert("another process's slot cannot be cleared", !_crypto.set_key(0, nullptr, nullptr, 0, idx));
		ut_assert("and stays held", !_crypto.set_key(0, nullptr, _key, sizeof(_key), idx));
	}

	PX4_INFO("session slots held by other processes: %d", held);
	return true;
}

bool SessionSlotTest::test_own_slot()
{
	for (uint8_t i = 0; i < CRYPTO_SESSION_KEY_COUNT; i++) {
		const uint8_t idx = CRYPTO_SESSION_KEY_FIRST + i;

		if (_crypto.set_key(0, nullptr, _key, sizeof(_key), idx)) {
			ut_assert("a slot this process set seals", seal(idx));
			ut_assert_true(_crypto.set_key(0, nullptr, nullptr, 0, idx));
			ut_assert("and after release seals nothing", !seal(idx));
			return true;
		}
	}

	ut_assert("a free slot exists", false);
	return false;
}

bool SessionSlotTest::run_tests()
{
	ut_assert_true(_crypto.open(CRYPTO_CHACHA20_POLY1305));
	ut_run_test(test_foreign_slots);
	ut_run_test(test_own_slot);
	_crypto.close();

	return (_tests_failed == 0);
}

ut_declare_test_c(test_session_slots, SessionSlotTest);
