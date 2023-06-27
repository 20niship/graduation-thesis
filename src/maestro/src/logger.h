#pragma once
static constexpr auto prefix_i = "";
static constexpr auto prefix_d = "\x1b[32m";
static constexpr auto prefix_w = "\x1b[33m";
static constexpr auto prefix_e = "\x1b[31m";

static constexpr auto suffix_e = "\x1b[0m";

static constexpr auto type_i = "[ INF ] ";
static constexpr auto type_d = "[ DBG ] ";
static constexpr auto type_w = "[ WAR ] ";
static constexpr auto type_e = "[ ERR ] ";
#if 1
#define LOGI std::cout << prefix_i << __FILE__ << " @ " << __LINE__ << type_i
#define LOGD std::cout << prefix_d << __FILE__ << " @ " << __LINE__ << type_d
#define LOGW std::cout << prefix_w << __FILE__ << " @ " << __LINE__ << type_w
#define LOGE std::cout << prefix_e << __FILE__ << " @ " << __LINE__ << type_e
#define LEND suffix_e << std::endl

#else

#define LOGI std::cout
#define LOGD std::cout
#define LOGW std::cout
#define LOGE std::cout
#define LEND std::endl
#endif
