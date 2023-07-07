#include <limits>

#include <cpptoml.h>

#include <iostream>
#include <spdlog/spdlog.h>

#include <hr4c/common.hpp>
#include <hr4c/core/ModbusClient.hpp>
#include <hr4c/core/h4rc.hpp>
#include <hr4c/core/logger.hpp>
#include <hr4c/graphics/gui.hpp>

std::tuple<std::string, int> get_ip_port(const std::string& tomlfile) {
  auto config = cpptoml::parse_file(tomlfile);
  auto socket = config->get_table("socket");
  auto ip     = socket->get_as<std::string>("ip");
  auto port   = socket->get_as<int>("port");
  if(!ip || !port) {
    std::cout << "Error: ip or port not found in " << tomlfile << std::endl;
    std::exit(1);
  }
  return std::make_tuple(*ip, *port);
}

//-----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
  auto fname      = (argc == 2) ? argv[1] : "../config.toml";
  auto [ip, port] = get_ip_port(fname);

  auto res = hr4c::start(ip, port);
  if(!res) {
    spdlog::error("Could not start server");
    hr4c::terminate();
    return 1;
  }

  struct AxisData {
    hr4c::AxisInterface axis;
    std::string name;
    hr4c::AxisBuffer buf;
    bool command      = false;
    float command_pos = 0;
    AxisData(int id) : axis(id), name("Axis" + std::to_string(id)) {}
  };

  std::vector<AxisData> axes;
  axes.emplace_back(hr4c::eAx1);
  axes.emplace_back(hr4c::eAx2);

  // axis.poweron();

  hr4c::init_view();
  bool loop = true;

  while(!kbhit() && loop) {
    for(auto& axis : axes) {
      axis.axis.update_sensor();
    }
    loop = hr4c::newframe_gui();
    {
      ImGui::Begin("Axis");
      for(auto& axis : axes) {
        if(ImGui::CollapsingHeader(axis.name.c_str())) {
          float pos = static_cast<double>(axis.axis.get_pos()) / 4096;
          float v   = static_cast<double>(axis.axis.get_vel()) / 4096;
          float c   = static_cast<double>(axis.axis.get_cur()) / 4096;
          axis.buf.add(pos, v, c);
          ImGui::Text("Pos, vel, cur  %f, %f, %f", pos, v, c);
          hr4c::plot_axis(axis.name, &axis.buf);
        }
      }
      ImGui::End();
    }

    ImGui::Begin("Menu");
    ImGui::Text("Axis Plotter");
    if(ImGui::CollapsingHeader("Power", true)) {
      ImGui::Text("Power");
      ImGui::SameLine();
      if(ImGui::Button("Poweron")) {
        spdlog::info("poweron start!!");
        for(auto& axis : axes) {
          axis.axis.poweron();
        }
      }
    }

    if(ImGui::CollapsingHeader("Power", true)) {
      ImGui::Text("parameters");
      int cur_limit = 400;
      float kp      = 0.1;
      float kd      = 0.1;
      float ki      = 0.3;
      ImGui::SliderInt("TorLimit", &cur_limit, 0, 5000);
      ImGui::SliderFloat("Kp", &kp, 0, 1);
      ImGui::SliderFloat("Kd", &kd, 0, 1);
      ImGui::SliderFloat("Ki", &ki, 0, 1);
      if(ImGui::Button("Set")) {
        spdlog::info("set pid parameter");
      }
    }

    if(ImGui::CollapsingHeader("Axis", true)) {
      ImGui::Text("Axis");
      ImGui::SameLine();
      int i = 0;
      for(auto& axis : axes) {
        i++;
        ImGui::Text("%s", axis.name.c_str());
        if(ImGui::Button(axis.name.c_str())) {
          spdlog::info("axis {} start!!", axis.name);
          // axis.axis.poweron();
        }
        static int homing = 1000;
        if(ImGui::Button("Home")) {
          homing = 0;
        }
        if(homing < 1000) {
          ImGui::SameLine();
          homing++;
          spdlog::info("home start!!");
          auto client = hr4c::ModbusClient::Get();
          client->set_command_data<int32_t>(hr4c::eCommand1, (int)axis.command_pos);
          client->set_command_data<int32_t>(hr4c::eCommand2, 1);
          client->set_command_data<int32_t>(hr4c::eCommand3, 1);
          client->send_command_data();
          ImGui::Text("Homing %d", homing);
        }

        if(ImGui::Button("Stop")) {
          spdlog::info("stop start!!");
          // axis.axis.stop();
        }
        ImGui::SameLine();
        if(ImGui::Button("Reset")) {
          spdlog::info("reset start!!");
          // axis.axis.reset();
        }
        ImGui::Separator();
        static int32_t default_pos = axis.axis.get_pos();
        if(ImGui::Checkbox("Command", &axis.command)) {
          default_pos = axis.axis.get_pos();
        }
        ImGui::DragFloat("Rotate", &axis.command_pos, 0.2);
        if(i == 1 && homing >= 990) {
          if(axis.command) {
            auto target = static_cast<int32_t>(axis.command_pos * 4096 + default_pos);
            ImGui::Text("Target %d", target);
            auto client = hr4c::ModbusClient::Get();
            client->set_command_data<int32_t>(hr4c::eCommand1, (int)axis.command_pos);
            client->set_command_data<int32_t>(hr4c::eCommand2, target);
            client->set_command_data<int32_t>(hr4c::eCommand3, 1);
            client->send_command_data();
          } else {
            auto client = hr4c::ModbusClient::Get();
            client->set_command_data<int32_t>(hr4c::eCommand1, (int)axis.command_pos);
            client->set_command_data<int32_t>(hr4c::eCommand2, 1);
            client->set_command_data<int32_t>(hr4c::eCommand3, -1);
            client->send_command_data();
          }
        }
      }
    }

    ImGui::Text("fps %f", ImGui::GetIO().Framerate);
    ImGui::Text("dt %f", ImGui::GetIO().DeltaTime);
    ImGui::End();

    hr4c::render_gui();
  }

  hr4c::terminate();
  return 0;
}
