// -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil; -*-

#ifndef FD_MUX_H_
#define FD_MUX_H_

#include <map>
#include <functional>
#include <vector>

// This needs a better name.
class FDMultiplexer {
public:
    // Handler for connections. Returns 'true' if we want to continue reading
    // or 'false' if we wish to be taken out of the multiplexer.
    typedef std::function<bool()> Handler;

    // These can only be set before Loop() is called or from a
    // running handler itself.
    // Returns false if that filedescriptor is already registered.
    bool RunOnReadable(int fd, const Handler &handler);
    bool IsRegisteredReadable(int fd) const;

    bool RunOnWritable(int fd, const Handler &handler);
    bool IsRegisteredWritable(int fd) const;

    bool RunOnException(int fd, const Handler &handler);
    bool IsRegisteredExceptioned(int fd) const;

    void ScheduleDelete(int fd);

    // Run the main loop. Blocks while there is still a filedescriptor
    // registered.
    void Loop();

private:
    std::map<int, Handler> r_handlers_;
    std::map<int, Handler> w_handlers_;
    std::map<int, Handler> e_handlers_;
    std::vector<int> to_delete_r_;
    std::vector<int> to_delete_w_;
    std::vector<int> to_delete_e_;
};
#endif // FD_MUX_H_
