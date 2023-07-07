#include <hr4c/graphics/gui.hpp>

int main() {
  hr4c::init_view();

  bool loop = true;
  while(loop) {
    loop = hr4c::newframe_gui();
    hr4c::update_gui();
    hr4c::render_gui();
  }

  hr4c::terminate_gui();
  return 0;
}
