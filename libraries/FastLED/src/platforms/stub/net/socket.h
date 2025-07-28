#pragma once

#ifdef FASTLED_HAS_NETWORKING

#include "fl/net/socket.h"
#include "fl/net/socket_factory.h"
#include "fl/vector.h"
#include "fl/deque.h"
#include "fl/string.h"
#include "fl/mutex.h"

namespace fl {

/// Stub socket implementation for testing (no actual network I/O)
class StubSocket : public Socket {
public:
    explicit StubSocket(const SocketOptions& options = {});
    ~StubSocket() override = default;
    
    // Socket interface implementation
    fl::future<SocketError> connect(const fl::string& host, int port) override;
    fl::future<SocketError> connect_async(const fl::string& host, int port) override;
    void disconnect() override;
    bool is_connected() const override;
    SocketState get_state() const override;
    
    fl::size read(fl::span<fl::u8> buffer) override;
    fl::size write(fl::span<const fl::u8> data) override;
    fl::size available() const override;
    void flush() override;
    
    bool has_data_available() const override;
    bool can_write() const override;
    void set_non_blocking(bool non_blocking) override;
    bool is_non_blocking() const override;
    
    void set_timeout(fl::u32 timeout_ms) override;
    fl::u32 get_timeout() const override;
    void set_keep_alive(bool enable) override;
    void set_nodelay(bool enable) override;
    
    fl::string remote_address() const override;
    int remote_port() const override;
    fl::string local_address() const override;
    int local_port() const override;
    
    SocketError get_last_error() const override;
    fl::string get_error_message() const override;
    
    bool set_socket_option(int level, int option, const void* value, fl::size value_size) override;
    bool get_socket_option(int level, int option, void* value, fl::size* value_size) override;
    
    int get_socket_handle() const override;
    
    /// Test control methods
    void set_mock_response(fl::span<const fl::u8> data);
    void set_mock_error(SocketError error, const fl::string& message = "");
    void set_mock_connection_delay(fl::u32 delay_ms);
    void simulate_connection_loss();
    void simulate_slow_network(fl::u32 bytes_per_second);
    
    /// Test inspection methods
    fl::vector<fl::u8> get_sent_data() const;
    fl::size get_bytes_sent() const;
    fl::size get_bytes_received() const;
    fl::size get_connection_attempts() const;
    
    /// Connect two stub sockets for loopback testing
    void connect_to_peer(fl::shared_ptr<StubSocket> peer);
    
protected:
    void set_state(SocketState state) override;
    void set_error(SocketError error, const fl::string& message = "") override;
    
private:
    const SocketOptions mOptions;
    SocketState mState = SocketState::CLOSED;
    SocketError mLastError = SocketError::SUCCESS;
    fl::string mErrorMessage;
    fl::string mRemoteHost;
    int mRemotePort = 0;
    fl::string mLocalAddress = "127.0.0.1";
    int mLocalPort = 0;
    int mSocketHandle = -1;
    bool mIsNonBlocking = false;
    fl::u32 mTimeout = 5000;
    
    // Mock data and peer connection
    fl::deque<fl::u8> mMockResponse;
    fl::deque<fl::u8> mSentData;
    fl::deque<fl::u8> mReceiveBuffer;
    fl::shared_ptr<StubSocket> mPeer;
    fl::u32 mConnectionDelay = 0;
    fl::u32 mBytesPerSecond = 0;  // 0 = unlimited
    
    // Test statistics
    fl::size mConnectionAttempts = 0;
    fl::size mBytesSent = 0;
    fl::size mBytesReceived = 0;
    
    // Mock behavior helpers
    void simulate_network_delay();
    fl::size calculate_transfer_rate(fl::size requested_bytes);
    void write_to_peer(fl::span<const fl::u8> data);
    static int generate_socket_handle();
};

// Platform-specific socket creation functions (required by socket_factory.cpp)
fl::shared_ptr<Socket> create_platform_socket(const SocketOptions& options);

// Platform capability queries  
bool platform_supports_ipv6();
bool platform_supports_tls();
bool platform_supports_non_blocking_connect();
bool platform_supports_socket_reuse();

} // namespace fl

#endif // defined(FASTLED_HAS_NETWORKING) && defined(FASTLED_STUB_IMPL) 
