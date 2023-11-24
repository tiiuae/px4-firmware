#include <arpa/inet.h>
#include <ctype.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#ifdef PX4_CRYPTO
#include <px4_platform_common/crypto.h>
using Crypto = PX4Crypto;
#else
#include "Crypto.hpp"
// Global keystore
uint8_t key_table_[256][crypto_aead_aes256gcm_KEYBYTES] = {0};
#endif // PX4_CRYPTO

#if !defined(MAVLINK_AES_SYM_KEY_IDX_START)
#define MAVLINK_AES_SYM_KEY_IDX_START 4
#endif // MAVLINK_AES_SYM_KEY_IDX_START
#if !defined(RSA_PUB_KEY_IDX)
#define RSA_PUB_KEY_IDX 2
#endif // RSA_PUB_KEY_IDX

namespace secure_mavlink_udp_proxy
{

constexpr size_t kUdpBufferLength{1024};
constexpr size_t kNonceLen{12};
constexpr size_t kAesMacSize{16};
constexpr size_t kRsaPacketSize{256      // RSA key size
	- 2 * 32 // 2* hash_function_output_size
	- 2};

// AES-GCM Operations
struct AesGcmOp {
	enum { Encrypt, Decrypt } op : 1;
	bool do_op(Crypto &aes_crypto, const uint8_t key_index, const uint8_t *indata,
		   const size_t indata_len, uint8_t *outdata,
		   size_t *outdata_len) const
	{
		if (op == Encrypt) {
			size_t nonce_len = kNonceLen;
			size_t mac_size = kAesMacSize;
			bool success =
				!(!aes_crypto.renew_nonce(NULL, kNonceLen) ||
				  !aes_crypto.get_nonce(outdata, &nonce_len) ||
				  !aes_crypto.encrypt_data(
					  key_index, indata, indata_len, outdata + nonce_len, outdata_len,
					  outdata + nonce_len + indata_len, &mac_size));
			*outdata_len += (nonce_len + mac_size);
			return success;

		} else {
			return (aes_crypto.renew_nonce(indata, kNonceLen) &&
				aes_crypto.decrypt_data(key_index, indata + kNonceLen,
							indata_len - kNonceLen - kAesMacSize,
							indata + kNonceLen + kAesMacSize,
							kAesMacSize, outdata, outdata_len));
		}
	}
};

// Header for key exchange
struct HeaderV0 {
	uint8_t version : 3;                 // 000 = this, 111 = reserved
	enum Type { REQ = 0, ACK } type : 1; // Type of this message: 0 = REQ, 1 = ACK
	enum SessionKeyType {                // Used session key type for MAVLink
		AES_GCM_256 = 0                    // 00 = AES-GCM 256
	} session_key_type : 2;              // 01, 10 and 11 reserved for further use
	/* uint8_t unused : 2;*/             //
	uint16_t key_number;                 // Send or acknowledged key number
	uint16_t mavlink_port;               // Listening MAVLink port number

	size_t serialize(uint8_t *buffer) const
	{
		buffer[0] = version | (type << 3) | (session_key_type << 4);
		buffer[1] = (key_number & 0xFF);
		buffer[2] = ((key_number >> 8) & 0xFF);
		buffer[3] = (mavlink_port & 0xFF);
		buffer[4] = ((mavlink_port >> 8) & 0xFF);
		return 5;
	}

	size_t deserialize(const uint8_t *buffer)
	{
		version = (buffer[0]) & 0x07;
		type = HeaderV0::Type((buffer[0] >> 3) & 0x01);
		session_key_type = SessionKeyType((buffer[0] >> 4) & 0x03);
		key_number = buffer[1] | (buffer[2] << 8);
		mavlink_port = buffer[3] | (buffer[4] << 8);
		return 5;
	}

	void print() const
	{
		printf("Header:\n"
		       "- version: %u\n"
		       "- type: %u\n"
		       "- session key type: %u\n"
		       "- mavlink_port: %u\n",
		       version, type, session_key_type, mavlink_port);
		uint8_t buffer[5];
		serialize(buffer);
		printf("- serialized bin | hex:\n");

		for (int i = 0; i < 5; i++) {
			printf("      %c%c%c%c %c%c%c%c  |  ", (buffer[i] & 0x80) ? '1' : '0',
			       (buffer[i] & 0x40) ? '1' : '0', (buffer[i] & 0x20) ? '1' : '0',
			       (buffer[i] & 0x10) ? '1' : '0', (buffer[i] & 0x08) ? '1' : '0',
			       (buffer[i] & 0x04) ? '1' : '0', (buffer[i] & 0x02) ? '1' : '0',
			       (buffer[i] & 0x01) ? '1' : '0');
			printf("%02X \n", buffer[i]);
		}
	}
};

// Coniguration structs

// Device configuration
struct Device {
	// Addess of UXV device (typically localhost)
	char address[INET_ADDRSTRLEN] {"127.0.0.1"};

	void print() const
	{
		printf("\n  Proxy:"
		       "\n  - address: %s"
		       "\n",
		       address);
	}
};

// QGroundControl side proxy configuration
struct QgcProxy {
	// Address of QGC Proxy
	char address[INET_ADDRSTRLEN];
	// Port where to send mavlink traffic
	uint16_t mavlink_port = 50909;
	// Port for session key exchange
	uint16_t key_port = 54320;

	void print() const
	{
		printf("\n  QgcProxy:"
		       "\n  - address: %s"
		       "\n  - mavlink_port: %u"
		       "\n  - key_port: %u"
		       "\n",
		       address,      //
		       mavlink_port, //
		       key_port);
	}
};

// Configuration of this device side proxy
struct Proxy {
	// Port where to listen session key exchange ACK
	uint16_t key_ack_port = 54321;
	// Port where to expect encrypted MAVLink from QGC proxy
	uint16_t qgc_mavlink_listen_port = 50910;
	// Port where to expect MAVLink packets from device
	uint16_t device_mavlink_listen_port = 12345;
	// How often key is renewed in seconds. 0 = No periodic key renewal
	unsigned key_change_interval = 3600;
	// How many seconds to wait for ACK
	unsigned ack_timeout = 5;
	// How many seconds to wait before trying to resend failed key
	unsigned resend_timeout = 5;
	// How many seconds inactivity in mavlink messages from QGC allowed before
	// changing the key
	unsigned mavlink_inactivity_timeout = 5;
	// If the header + encrypted key is encrypted again in key exchange
	bool encrypted_key_exchange_packet = false;
	// File to look RSA public key (In case of PX4Crypto the keystore is used
	// instead)
	char rsa_public_key_path[200];
	// Print information of sent and received pacekts
	bool verbose = false;

	void print() const
	{
		printf("\n  Proxy:"
		       "\n  - key_ack_port: %u"
		       "\n  - qgc_mavlink_listen_port: %u"
		       "\n  - device_mavlink_listen_port: %u"
		       "\n  - key_change_interval: %d"
		       "\n  - ack_timeout: %d"
		       "\n  - resend_timeout: %d"
		       "\n  - mavlink_inactivity_timeout: %d"
		       "\n  - encrypted_key_exchange_packet: %s"
		       "\n  - rsa_public_key_path: %s"
		       "\n  - verbose: %s"
		       "\n",
		       key_ack_port,                                       //
		       qgc_mavlink_listen_port,                            //
		       device_mavlink_listen_port,                         //
		       key_change_interval,                                //
		       ack_timeout,                                        //
		       resend_timeout,                                     //
		       mavlink_inactivity_timeout,                         //
		       (encrypted_key_exchange_packet ? "true" : "false"), //
		       rsa_public_key_path,                                //
		       (verbose ? "true" : "false"));
	}
};

struct Configuration {
	Device device;
	QgcProxy qgc_proxy;
	Proxy proxy;

	void print() const
	{
		printf("\nMAVLink UDP Device proxy configuration"
		       "\n--------------------------------------\n");
		device.print();
		qgc_proxy.print();
		proxy.print();
		printf("\n======================================\n");
	}
};

/// @brief Remove spaces and quotes in the beginning and the end of c-string
/// @param[in, out] str c-string to be processed
void trimAndRemoveQuotes(char *str)
{
	const size_t len{strlen(str)};
	size_t start{0};
	size_t end{len - 1};

	while (start < len && isspace((unsigned char)str[start])) {
		start++;
	}

	while (end >= start && isspace((unsigned char)str[end])) {
		end--;
	}

	if (str[start] == '"' && str[end] == '"') {
		start++;
		end--;
	}

	if (start > 0 || end < len - 1) {
		memmove(str, str + start, end - start + 1);
		str[end - start + 1] = '\0';
	}
}

/// @brief Split line into key value pair
/// @param[in]  line  input text line
/// @param[out] key   parsed key
/// @param[out] value parsed value
void splitKeyValue(char *line, char *key, char *value)
{
	// Remove comments
	char *comment{strchr(line, '#')};

	if (comment) {
		*comment = '\0';
	}

	// Split to key and value
	char *sep{strchr(line, ':')};

	if (sep) {
		*sep = '\0';
		strcpy(key, line);
		strcpy(value, sep + 1);
		trimAndRemoveQuotes(key);
		trimAndRemoveQuotes(value);
	}
}

/// @brief Parse Configuration struct from
/// @param[in]  filename  file name to parse
/// @return Configuration struct with parsed information
/// @todo This is simple YAML parser without proper error handling and
/// checks in place.
Configuration parse_yaml(const char *filename)
{

	// Open file
	FILE *config_file{fopen(filename, "r")};

	if (config_file == NULL) {
		fprintf(stderr, "Cannot open configuration file: %s\n", filename);
		exit(-1);
	}

	// Create and zero initialize config structure
	Configuration config{};

	char key[256];
	char value[256];
	char *line{NULL};
	size_t line_len{0};
	bool in_device{false};
	bool in_qgc_proxy{false};
	bool in_proxy{false};

	while (getline(&line, &line_len, config_file) != -1) {
		splitKeyValue(line, key, value);

		if (strcmp(key, "device") == 0) {
			in_device = true;
			in_qgc_proxy = in_proxy = false;

		} else if (strcmp(key, "qgc_proxy") == 0) {
			in_qgc_proxy = true;
			in_device = in_proxy = false;

		} else if (strcmp(key, "proxy") == 0) {
			in_device = in_qgc_proxy = false;
			in_proxy = true;
		}

		// Device
		else if (in_device) {
			if (strcmp(key, "address") == 0) {
				strcpy(config.device.address, value);
			}
		}

		// QGC Proxy
		else if (in_qgc_proxy) {
			if (strcmp(key, "address") == 0) {
				strcpy(config.qgc_proxy.address, value);

			} else if (strcmp(key, "mavlink_port") == 0) {
				config.qgc_proxy.mavlink_port = atoi(value);

			} else if (strcmp(key, "key_port") == 0) {
				config.qgc_proxy.key_port = atoi(value);
			}
		}

		// This proxy
		else if (in_proxy) {
			if (strcmp(key, "key_ack_port") == 0) {
				config.proxy.key_ack_port = atoi(value);

			} else if (strcmp(key, "qgc_mavlink_listen_port") == 0) {
				config.proxy.qgc_mavlink_listen_port = atoi(value);

			} else if (strcmp(key, "device_mavlink_listen_port") == 0) {
				config.proxy.device_mavlink_listen_port = atoi(value);

			} else if (strcmp(key, "key_change_interval") == 0) {
				config.proxy.key_change_interval = atoi(value);

			} else if (strcmp(key, "ack_timeout") == 0) {
				config.proxy.ack_timeout = atoi(value);

			} else if (strcmp(key, "resend_timeout") == 0) {
				config.proxy.resend_timeout = atoi(value);

			} else if (strcmp(key, "mavlink_inactivity_timeout") == 0) {
				config.proxy.mavlink_inactivity_timeout = atoi(value);

			} else if (strcmp(key, "encrypted_key_exchange_packet") == 0) {
				config.proxy.encrypted_key_exchange_packet =
					(strcmp(value, "true") == 0);

			} else if (strcmp(key, "verbose") == 0) {
				config.proxy.verbose = (strcmp(value, "true") == 0);

			} else if (strcmp(key, "rsa_public_key_path") == 0) {
				strcpy(config.proxy.rsa_public_key_path, value);
			}
		}
	}

	fclose(config_file);
	free(line);
	return config;
} // parse_yaml

// Key request signalling
bool new_key_requested{true};
pthread_mutex_t key_request_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t condition = PTHREAD_COND_INITIALIZER;

// Cipher & key number

// We need two indices for storing the aes-gcm key. The currently used
// and the new still being exchanged. At some point the exchanged one
// becomes current. This is to keep track which one is curent/referred.
// Possible values are MAVLINK_AES_SYM_KEY_IDX_START and
// MAVLINK_AES_SYM_KEY_IDX_START + 1
uint8_t aes_key_index{MAVLINK_AES_SYM_KEY_IDX_START};
// Latest exchanged key_number
uint16_t aes_key_number{0};
// Mutex for protecting currently used AES-GCM key number and corresponding
// index in store
pthread_mutex_t key_switch_mutex = PTHREAD_MUTEX_INITIALIZER;

/// @brief Configure Internet socket (sockaddr_in)
/// @param[in]  address_str IPv4 address in c-string e.g. "192.168.0.1"
/// @param[in]  port        IPv4 port number
/// @param[out] addr        configured sockaddr_in struct
/// @return true if configuration was successful
bool configure_addr(const char *address_str, const uint16_t port,
		    struct sockaddr_in *addr)
{
	*addr = {};
	addr->sin_family = AF_INET;
	addr->sin_port = htons(port);

	if (address_str == NULL) {
		addr->sin_addr.s_addr = INADDR_ANY;

	} else if (inet_aton(address_str, &(addr->sin_addr)) == 0) {
		printf("Invalid IP address\n");
		return false;
	}

	return true;
}

/// @brief Open UDP socket with optional timeout value
/// @param[in]  timeout_s socket timeout in seconds
/// @return socket file descriptor
int open_udp_socket(const unsigned timeout_s)
{
	int sock{socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)};

	if (sock == -1) {
		perror("Cannot create UDP socket\n");
		exit(-1);
	}

	if (timeout_s > 0) {
		struct timeval timeout_tv = {static_cast<time_t>(timeout_s), 0};

		if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout_tv,
			       sizeof(timeout_tv)) < 0) {
			perror("Error setting timeout");
			close(sock);
			exit(-1);
		}
	}

	return sock;
}

/// @brief Open & bind UDP socket for listening with optional timeout value
/// @param[in]  timeout_s socket timeout in seconds
/// @param[in]  listening_port port bind for listening
/// @param[out] addr configured sockaddr_in struct for listening socket
/// @return socket file descriptor
int open_listening_udp_socket(const unsigned timeout_s,
			      const uint16_t listening_port,
			      struct sockaddr_in *addr)
{
	// Reveiver configuration:
	configure_addr(NULL, listening_port, addr);
	int sock{open_udp_socket(timeout_s)};

	// bind socket
	if (bind(sock, (sockaddr *)addr, sizeof(sockaddr)) == -1) {
		perror("Receiver socket bind failed.");
		close(sock);
		exit(-1);
	}

	return sock;
}

/// @brief Function for signalling and clearing key request
/// @param[in]  state   state wanted to be set/signalled
/// @param[in]  printer optional prefix for for printing
void set_new_key_request(const bool state, const char *printer = NULL)
{
	pthread_mutex_lock(&key_request_mutex);
	const bool prev_state{new_key_requested};
	new_key_requested = state;

	if (state == true && prev_state == false) {
		pthread_cond_signal(&condition);

		if (printer) {
			printf("%sRequesting a new key.\n", printer);
		}
	}

	pthread_mutex_unlock(&key_request_mutex);
}

/// @brief Thread for triggering periodic key exchanges
/// @param[in]  arg thread argument of type Configuration
void *timer_thread(void *arg)
{
	const long key_change_interval_s{
		static_cast<Configuration *>(arg)->proxy.key_change_interval};

	if (key_change_interval_s <= 0) {
		return NULL;
	}

	while (true) {
		sleep(key_change_interval_s);
		set_new_key_request(true, "timer: ");
	}
}

/// @brief Thread for performing key exchanges
/// @param[in]  arg thread argument of type Configuration.
void *key_exchange_thread(void *arg)
{
	const Configuration conf{*static_cast<Configuration *>(arg)};

#ifdef PX4_CRYPTO
	PX4Crypto aes_crypto;
	PX4Crypto rsa_crypto;

	if (!rsa_crypto.open(CRYPTO_RSA_OAEP) || !aes_crypto.open(CRYPTO_AES)) {
		perror("Error opening crypto.");
		exit(-1);
	}

#else
	// SodiumCrypto aes_crypto;
	// SodiumCrypto rsa_crypto; // Sodium doesn't support RSA
	CryptoppCrypto rsa_crypto(conf.proxy.rsa_public_key_path);
	CryptoppCrypto aes_crypto;

	if (!rsa_crypto.open(CRYPTO_RSA_OAEP) || !aes_crypto.open(CRYPTO_AES)) {
		perror("Error opening crypto.");
		exit(-1);
	}

#endif

	// Open UDP socket
	struct sockaddr_in ack_addr, key_addr;
	int key_exchange_socket{open_listening_udp_socket(
					conf.proxy.ack_timeout, conf.proxy.key_ack_port, &ack_addr)};

	// Sender configuration:
	if (!configure_addr(conf.qgc_proxy.address, conf.qgc_proxy.key_port,
			    &key_addr)) {
		perror("Invalid IP address");
		close(key_exchange_socket);
		exit(-1);
	}

	bool skip_sending{false};

	// Reusable buffer for key sending
	uint8_t *key_packet{static_cast<uint8_t *>(malloc(kUdpBufferLength))};
	// Reusable Header for key exchange
	HeaderV0 key_packet_header{
		.type{HeaderV0::Type::REQ},
		.session_key_type{HeaderV0::SessionKeyType::AES_GCM_256},
		.key_number{0},
		.mavlink_port{conf.proxy.qgc_mavlink_listen_port}};
	// Reusable buffer for receiving ACK
	uint8_t *buf{static_cast<uint8_t *>(malloc(kUdpBufferLength))};
	// Reusable Header for ACK
	HeaderV0 ack_header;

	while (true) { // loop

		pthread_mutex_lock(&key_request_mutex);

		while (!new_key_requested) { // wait for signal
			pthread_cond_wait(&condition, &key_request_mutex);
		}

		pthread_mutex_unlock(&key_request_mutex);
		const uint8_t new_aes_key_index =
			MAVLINK_AES_SYM_KEY_IDX_START +
			(aes_key_index - MAVLINK_AES_SYM_KEY_IDX_START + 1) % 2;

		// generate and send AES-GCM key
		if (!skip_sending) {
			// Generate packet for key exchange header + encrypted aes_gcm_key
			// Update key number in header
			++key_packet_header.key_number;
			// Serialize header into packet
			size_t key_packet_current_len{key_packet_header.serialize(key_packet)};
			// Generate a new key into packet
			{
				const clock_t begin{clock()};
				// Generate a new symmetric AES key
				aes_crypto.generate_key(new_aes_key_index, false);
				// Get encrypted key for sending
				size_t key_size{256};
				uint8_t encrypted_key[256] {0};

				if (!rsa_crypto.get_encrypted_key(new_aes_key_index, encrypted_key,
								  &key_size, RSA_PUB_KEY_IDX)) {
					fprintf(stderr, "key_exchange: Couldn't get encrypted session key\n");
					sleep(conf.proxy.resend_timeout);
					continue;
				}

				memcpy(key_packet + key_packet_current_len, encrypted_key, key_size);
				key_packet_current_len += key_size;

				if (conf.proxy.encrypted_key_exchange_packet) {
					// Encrypt header + encrypted key
					size_t encrypted_key_packet_current_len{0};
					uint8_t *encrypted_key_packet{
						static_cast<uint8_t *>(malloc(kUdpBufferLength))};
					/*const size_t packet_size = get_min_blocksize(RSA_PUB_KEY_IDX);*/
					const size_t packets{(key_packet_current_len + kRsaPacketSize - 1) /
							     kRsaPacketSize};
					size_t packet_left{key_packet_current_len};
					size_t encrypted_chunk_len{kUdpBufferLength};

					for (size_t i{0}; i < packets; ++i) {
						size_t chunk_size{
							(packet_left < kRsaPacketSize ? packet_left : kRsaPacketSize)};
						rsa_crypto.encrypt_data(
							RSA_PUB_KEY_IDX, key_packet + i * kRsaPacketSize, chunk_size,
							encrypted_key_packet + encrypted_key_packet_current_len,
							&encrypted_chunk_len, NULL, 0);
						packet_left -= chunk_size;
						encrypted_key_packet_current_len += encrypted_chunk_len;
						// buffer availability for next iteration
						encrypted_chunk_len =
							kUdpBufferLength - encrypted_key_packet_current_len;
					}

					memcpy(key_packet, encrypted_key_packet,
					       encrypted_key_packet_current_len);
					key_packet_current_len = encrypted_key_packet_current_len;
					free(encrypted_key_packet);
				}

				const clock_t end{clock()};
				const double dt{1000. * (double)(end - begin) / CLOCKS_PER_SEC};
				printf("key_exchange: Trying to exchange AES-GCM key #%u, RSA "
				       "encryption duration %.3lf ms.\n",
				       key_packet_header.key_number, dt);
			}

			key_packet_header.print();

			// Send packet to QGC proxy using UDP
			ssize_t bytes_send{sendto(key_exchange_socket, key_packet,
						  key_packet_current_len, 0,
						  (sockaddr *)&key_addr, sizeof(key_addr))};

			if (bytes_send <= 0) {
				perror("key_exchange: Failed to send encrypted key.\n");
			}

		} else {
			skip_sending = false;
		}

		// Wait for ACK
		printf("key_exchange: Waiting ACK in port %d..\n", conf.proxy.key_ack_port);
		socklen_t slen{INET_ADDRSTRLEN};
		const ssize_t bytes_read{recvfrom(key_exchange_socket, buf,
						  kUdpBufferLength, 0,
						  (struct sockaddr *)&ack_addr, &slen)};

		if (bytes_read < 256) {
			fprintf(stderr,
				"key_exchange: Couldn't receive valid ACK, bytes_read = %ld\n",
				bytes_read);
			sleep(conf.proxy.resend_timeout);
			continue;
		}

		// Deserialize header
		const size_t header_len{ack_header.deserialize(buf)};
		ack_header.print();

		if (ack_header.version != 0) {
			fprintf(stderr, "key_exchange: received invalid version %u\n",
				ack_header.version);
			continue;
		}

		if (ack_header.type != HeaderV0::Type::ACK) {
			fprintf(stderr, "key_exchange: received invalid packet type. Expected "
				"ACK, received REQ\n");
			continue;
		}

		if (ack_header.session_key_type != HeaderV0::SessionKeyType::AES_GCM_256) {
			fprintf(stderr,
				"key_exchange: QGC Proxy signalled invalid session key "
				"type. Expected AES-GCM 256, received UNKNOWN (%u).\n",
				ack_header.session_key_type);
			continue;
		}

		// Get signature
		const uint8_t *signature{buf + header_len};

		// Check ACKed key number matches our key number
		if (ack_header.key_number != key_packet_header.key_number) {
			fprintf(stderr,
				"key_exchange: Received ACK with incorrect key number %u "
				"expecting %u\n.",
				ack_header.key_number, key_packet_header.key_number);

			if (ack_header.key_number < key_packet_header.key_number) {
				fprintf(stderr, "key_exchange: Waiting for fresher ACK.\n");
				// Continue waiting for a new ACK for another timeout.
				skip_sending = true;
				continue;
			}

			sleep(conf.proxy.resend_timeout);
			continue;
		}

		const clock_t begin{clock()};

		if (!rsa_crypto.signature_check(RSA_PUB_KEY_IDX, signature, buf,
						header_len)) {
			fprintf(stderr, "key_exchange: Signature verification failed.\n");
			sleep(conf.proxy.resend_timeout);
			continue;
		}

		const clock_t end{clock()};
		const double dt{1000. * (double)(end - begin) / CLOCKS_PER_SEC};
		// Update the new key index and number.
		{
			pthread_mutex_lock(&key_switch_mutex);
			aes_key_index = new_aes_key_index;
			aes_key_number = ack_header.key_number;
			pthread_mutex_unlock(&key_switch_mutex);
		}
		printf("key_exchange: Received valid ACK for AES-GCM key # %u, RSA "
		       "verification duration %.3lf ms\n",
		       ack_header.key_number, dt);
		// Clear the new key request
		set_new_key_request(false, "key_exchange: ");
	} // loop

	free(key_packet);
	free(buf);
	close(key_exchange_socket);
}

/// @struct Parameters to be passed on MAVLink pthread
struct MavlinkThreadArg {
	int listen_socket;            /// <- socket or listening packet
	in_addr_t expected_addr;      /// <- expected address for incoming packets
	int send_socket;              /// <- socket used for sending packet forward
	struct sockaddr_in dest_addr; /// <- destination address struct
	const char *prefix;           /// <- prefix for prints
	AesGcmOp aes_op;              /// <- encrypt or decrypt received message
	bool notify;                  /// <- whether to signal new key exchange
	bool verbose;                 /// <- print summary for each message
};

/// @brief MAVLink thread: Listen data from one UDP socket, encrypt/decrypt
/// it and forrward it into another UDP socket
/// @param[in]  arg thread argument of type MavLinkThreadArgs
void *listen_and_send(void *arg)
{
	// Get parameters from args
	char printer[128];
	strncpy(printer, static_cast<MavlinkThreadArg *>(arg)->prefix,
		sizeof(printer) - 1);
	printer[sizeof(printer) - 1] = '\0';
	strcat(printer, ": ");
	int listen_socket{static_cast<MavlinkThreadArg *>(arg)->listen_socket};
	int send_socket{static_cast<MavlinkThreadArg *>(arg)->send_socket};
	const struct sockaddr_in dest_addr {
		static_cast<MavlinkThreadArg *>(arg)->dest_addr
	};
	char dest_addr_str[INET_ADDRSTRLEN + 6];
	inet_ntop(AF_INET, &(dest_addr.sin_addr), dest_addr_str, INET_ADDRSTRLEN);
	sprintf(dest_addr_str + strlen(dest_addr_str), ":%d",
		ntohs(dest_addr.sin_port));
	const AesGcmOp aes_op{static_cast<MavlinkThreadArg *>(arg)->aes_op};
	const in_addr_t expected_addr{
		static_cast<MavlinkThreadArg *>(arg)->expected_addr};
	char expected_ip_str[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &(expected_addr), expected_ip_str, INET_ADDRSTRLEN);
	const bool verbose{static_cast<MavlinkThreadArg *>(arg)->verbose};
	const bool notify{static_cast<MavlinkThreadArg *>(arg)->notify};

// Create and open AES crypto
#ifdef PX4_CRYPTO
	PX4Crypto aes_crypto;
#else
	CryptoppCrypto aes_crypto;
#endif

	if (!aes_crypto.open(CRYPTO_AES)) {
		fprintf(stderr, "%sError opening crypto.\n", printer);
		exit(-1);
	}

	// Resources for receiving
	uint8_t *buf{static_cast<uint8_t *>(malloc(kUdpBufferLength))};
	socklen_t slen{INET_ADDRSTRLEN};
	struct sockaddr_in source_addr;
	bool first_mavlink_packet{true};
	bool resume_mavlink{false};
	printf("%sListening fd: %d\n", printer, listen_socket);
	const char *source =
		(aes_op.op == AesGcmOp::Encrypt) ? "Local" : "Remote secure";
	uint32_t consecutive_recv_failures{0};
	char source_addr_str[INET_ADDRSTRLEN + 6] {0};

	// Resources for forwarding
	uint8_t *data_to_send{static_cast<uint8_t *>(malloc(kUdpBufferLength))};

	// Get initial values for key index and number
	uint16_t key_number{0};
	uint8_t key_index{MAVLINK_AES_SYM_KEY_IDX_START};

	// Local variable to check if key has been set (no need to lock)
	bool key_ready{false};

	size_t data_len{kUdpBufferLength};
	ssize_t bytes_read{0};

	while (true) {
		// Listen for new data
		data_len = kUdpBufferLength;
		bytes_read = recvfrom(listen_socket, buf, kUdpBufferLength, 0,
				      (sockaddr *)&source_addr, &slen);

		if (bytes_read <= 0) {
			if (consecutive_recv_failures == 0 && !first_mavlink_packet) {
				fprintf(stderr, "%s %s MAVLink communication stopped.\n", printer,
					source);
			}

			consecutive_recv_failures += 1;

			if (notify) {
				set_new_key_request(true, printer);
			}

			resume_mavlink = true;
			continue;
		}

		consecutive_recv_failures = 0;

		if (verbose || source_addr.sin_addr.s_addr != expected_addr) {
			// Format source address to c-string
			inet_ntop(AF_INET, &(source_addr.sin_addr), source_addr_str,
				  INET_ADDRSTRLEN);
			sprintf(source_addr_str + strlen(source_addr_str), ":%d",
				ntohs(source_addr.sin_port));

			// Check if data is from expected IP
			if (source_addr.sin_addr.s_addr != expected_addr) {
				fprintf(stderr,
					"%sWrong IP. Expected MAVLink data from %s (%u), received data "
					"from %s (%u)\n",
					printer, expected_ip_str, expected_addr, source_addr_str,
					source_addr.sin_addr.s_addr);
				continue;
			}
		}

		const clock_t begin{clock()};

		// In case of encrypt always use latest key.
		// When decrypting try to switch key only after decrypt failure.
		if (!key_ready || aes_op.op == AesGcmOp::Encrypt) {
			pthread_mutex_lock(&key_switch_mutex);

			if (!(key_number = aes_key_number)) {
				// no key set (key number == 0)
				pthread_mutex_unlock(&key_switch_mutex);
				continue;
			}

			key_index = aes_key_index;
			pthread_mutex_unlock(&key_switch_mutex);
			key_ready = true;
		}

		if (!aes_op.do_op(aes_crypto, key_index, buf, bytes_read, data_to_send,
				  &data_len)) {
			// check key number
			pthread_mutex_lock(&key_switch_mutex);

			if (key_number == aes_key_number) {
				// Key was not updated
				pthread_mutex_unlock(&key_switch_mutex);
				continue;
			}

			// update local key number and index
			key_number = aes_key_number;
			key_index = aes_key_index;
			pthread_mutex_unlock(&key_switch_mutex);

			// retry with new key
			if (!aes_op.do_op(aes_crypto, key_index, buf, bytes_read, data_to_send,
					  &data_len)) {
				fprintf(stderr, "Data couldn't be %scrypted with key#%u\n",
					(aes_op.op == AesGcmOp::Encrypt ? "en" : "de"), key_number);
				continue;
			}
		}

		const clock_t end{clock()};
		const double dt{1000. * (double)(end - begin) / CLOCKS_PER_SEC};

		const ssize_t bytes_send{sendto(send_socket, data_to_send, data_len, 0,
						(sockaddr *)&dest_addr, slen)};

		if (bytes_send <= 0) {
			///@todo print also errno, error
			fprintf(stderr, "%sError forwarding MAVLink message.\n", printer);

		} else {
			if (first_mavlink_packet || resume_mavlink) {
				printf("%sSecure MAVLink communication %s. (Key #%u)\n", printer,
				       (first_mavlink_packet ? "started" : "resumed"), key_number);
				first_mavlink_packet = resume_mavlink = false;
			}

			if (verbose) {
				char aes_string[128] {0};

				if (aes_op.op == AesGcmOp::Encrypt) {
					snprintf(aes_string, 127, "-> encrypt: %.3lf ms =>", dt);

				} else {
					snprintf(aes_string, 127, "=> decrypt: %.3lf ms ->", dt);
				}

				fprintf(stderr, "%srecv[%s]: %ld B %s send[%s]: %ld B\n", printer,
					source_addr_str, bytes_read, aes_string, dest_addr_str,
					bytes_send);
			}
		}
	}

	{ fprintf(stderr, "%sWhile loop exited!!.\n", printer); }

	free(buf);
	free(data_to_send);
}
} // namespace secure_mavlink_udp_proxy

#ifdef PX4_CRYPTO
extern "C" __EXPORT int secure_mavlink_udp_proxy_main()
{
#else
int main()
{
#endif
	using namespace secure_mavlink_udp_proxy;
	// Threads
	pthread_t key_exchange, remote_send, remote_recv, timer;

	// Parse config
	const Configuration conf{parse_yaml("config.yaml")};
	conf.print();

	// key exchange thread
	const Configuration *key_config{&conf};
	pthread_create(&key_exchange, NULL, &key_exchange_thread, (void *)key_config);

	struct sockaddr_in device_addr, source_addr, qgc_addr, dest_addr;

	int device_mavlink_socket{open_listening_udp_socket(
					  conf.proxy.mavlink_inactivity_timeout,
					  conf.proxy.device_mavlink_listen_port, &device_addr)};

	int qgc_mavlink_listen_socket{
		open_listening_udp_socket(conf.proxy.mavlink_inactivity_timeout,
					  conf.proxy.qgc_mavlink_listen_port, &qgc_addr)};

	configure_addr(conf.qgc_proxy.address, conf.qgc_proxy.mavlink_port,
		       &dest_addr);

	// Get the first MAVLink packet from device to find out used port
	printf("Mavlink: Waiting device to connect... \n");
	uint8_t *buf{static_cast<uint8_t *>(malloc(kUdpBufferLength))};
	size_t bytes_read{0};
	socklen_t slen{INET_ADDRSTRLEN};
	in_addr_t expected_addr{inet_addr(conf.device.address)};

	while (true) {
		bytes_read = recvfrom(device_mavlink_socket, buf, kUdpBufferLength, 0,
				      (sockaddr *)&source_addr, &slen);

		// Check that received packet was MAVLink packet
		if (bytes_read < 8 || !(buf[0] == 0xFE || buf[0] == 0xFD)) {
			fprintf(stderr,
				"Mavlink: Didn't receive valid MAVLink message."
				"(Bytes read = %ld, slen = %u)\n",
				bytes_read, slen);
			continue;
		}

		// Check that packet come from expected device
		if (source_addr.sin_addr.s_addr != expected_addr) {
			char source_ip_str[INET_ADDRSTRLEN];
			inet_ntop(AF_INET, &(source_addr.sin_addr), source_ip_str,
				  INET_ADDRSTRLEN);
			fprintf(stderr,
				"Wrong IP. Expected MAVLink data from %s (%d), received data "
				"from %s (%d)\n",
				conf.device.address, device_addr.sin_addr.s_addr, source_ip_str,
				source_addr.sin_addr.s_addr);
			continue;
		}

		printf("Mavlink: Device connected. MAVLink reply port %u set.\n",
		       ntohs(source_addr.sin_port));
		break;
	}

	free(buf);

	struct MavlinkThreadArg *rsend_args {
		static_cast<struct MavlinkThreadArg *>(malloc(sizeof(MavlinkThreadArg)))
	};
	rsend_args->aes_op.op = AesGcmOp::Encrypt;
	rsend_args->prefix = "remote_send";
	rsend_args->dest_addr = dest_addr;
	rsend_args->expected_addr = inet_addr(conf.device.address);
	rsend_args->listen_socket = device_mavlink_socket;
	rsend_args->notify = false;
	rsend_args->send_socket = qgc_mavlink_listen_socket;
	rsend_args->verbose = conf.proxy.verbose;

	struct MavlinkThreadArg *rrecv_args {
		static_cast<struct MavlinkThreadArg *>(malloc(sizeof(MavlinkThreadArg)))
	};
	rrecv_args->aes_op.op = AesGcmOp::Decrypt;
	rrecv_args->prefix = "remote_recv";
	rrecv_args->dest_addr = source_addr;
	rrecv_args->expected_addr = inet_addr(conf.qgc_proxy.address);
	rrecv_args->listen_socket = qgc_mavlink_listen_socket;
	rrecv_args->notify = true;
	rrecv_args->send_socket = device_mavlink_socket;
	rrecv_args->verbose = conf.proxy.verbose;

	pthread_create(&remote_send, NULL, &listen_and_send,
		       static_cast<void *>(rsend_args));
	pthread_create(&remote_recv, NULL, &listen_and_send,
		       static_cast<void *>(rrecv_args));
	Configuration c = conf;
	pthread_create(&timer, NULL, &timer_thread, static_cast<void *>(&c));

	pthread_join(timer, NULL);
	pthread_join(key_exchange, NULL);
	pthread_join(remote_send, NULL);
	pthread_join(remote_recv, NULL);
	free(rsend_args);
	free(rrecv_args);
	close(device_mavlink_socket);
	close(qgc_mavlink_listen_socket);
	return 0;
}
