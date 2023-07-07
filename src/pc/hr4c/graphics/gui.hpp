#pragma once
#include <cmath>
#include <imgui.h>
#include <implot.h>
#include <iostream>

namespace hr4c {

int init_view();
bool newframe_gui();
void update_gui();
void render_gui();
void terminate_gui();

struct ScrollingBuffer {
  int MaxSize;
  int Offset;
  ImVector<ImVec2> Data;
  ScrollingBuffer(int max_size = 2000) {
    MaxSize = max_size;
    Offset  = 0;
    Data.reserve(MaxSize);
  }
  void AddPoint(float x, float y) {
    if(Data.size() < MaxSize)
      Data.push_back(ImVec2(x, y));
    else {
      Data[Offset] = ImVec2(x, y);
      Offset       = (Offset + 1) % MaxSize;
    }
  }
  void Erase() {
    if(Data.size() > 0) {
      Data.shrink(0);
      Offset = 0;
    }
  }
};

struct AxisBuffer {
  ScrollingBuffer sd_pos, sd_vel, sd_tor;
  float t = 0;
  AxisBuffer(int max_size = 1000) {
    sd_pos = ScrollingBuffer(max_size);
    sd_vel = ScrollingBuffer(max_size);
    sd_tor = ScrollingBuffer(max_size);
  }
  void add(float pos, float vel, float tor) {
    t += ImGui::GetIO().DeltaTime;
    sd_pos.AddPoint(t, pos);
    sd_vel.AddPoint(t, vel);
    sd_tor.AddPoint(t, tor);
  }
};

void plot_axis(const std::string& name, const AxisBuffer* data);

} // namespace hr4c
