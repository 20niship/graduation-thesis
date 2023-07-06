#pragma once

#include <MMCConnection.h>
#include <MMCSingleAxis.h>
#include <MMC_definitions.h>
#include <MMC_host_comm_API.h>

#include <iostream>

inline std::string get_cmmc_exception_error_message(const CMMCException& excp) {
  const auto error = excp.error();
  switch(error) {
    case 0: return "Everything is OK, no errors and warnings";
    case -1: return "NC driver error";
    case -2: return "NC driver memory mapping failure";
    case -3: return "Record length is larger than the maximal recorder buffer size or input arguments to MMC_UploadDataCmd function are not valid";
    case -4: return "Illegal Record Gap value";
    case -5: return "There is a call to begin recording when the Recorder is already operating";
    case -6: return "Incorrect Node handle (axis or group of axes)";
    case -7: return "The Node's function blocks' list is empty";
    case -8: return "Node state is unsuitable for the active function block";
    case -9: return "Node is not found";
    case -10: return "You are trying to perform an operation that cannot operate from Disabled mode";
    case -11: return "No free Nodes in the list";
    case -12: return "The node type is incorrect";
    case -13: return "Incorrect function block type parameter";
    case -14: return "Free function blocks list is empty";
    case -15: return "Function block type not supported by Maestro";
    case -16: return "Function block pointer not found";
    case -18: return "The function block Handle validity check failed";
    case -19: return "The function block is already removed from the active list and marked as free";
    case -21: return "Axis used by other group";
    case -22: return "One of the parameters is out of the permitted range";
    case -23: return "Axis Update Cycle Period or Shift is not compatible for the group";
    case -24: return "Incorrect coordinate system was used or the coordinate system is not enabled at the present";
    case -25: return "Group cannot receive motion command because it's disabled";
    case -26:
      return "Inappropriate connection status";
      // Add more cases for other error IDs here
    case -28: return "Cannot open a UDP socket";
    case -29: return "Open UDP socket fail";
    case -30: return "Bind UDP socket fail";
    case -31: return "Calling this function from an IPC connection is not permitted";
    case -32: return "Incorrect connection type parameter";
    case -33: return "Cannot perform any of the operations listed in the Resolution when at least one of the axis is powered on";
    case -34: return "Update of the UDP socket on-connection table fails";
    case -35: return "Update of the UDP address on-connection table fails";
    case -36: return "Update of the event mask on-connection table fails";
    case -37: return "Get UDP socket from connection table fails";
    case -39: return "Cannot read event mask";
    case -40: return "Cannot operate this specific function in the current mode";
    case -43: return "Set DHCP function sent with the wrong input parameter";
    case -44: return "Internal process failure";
    case -45: return "Failed to read the download version status from the flash params table";
    case -46: return "Failed to recreate the configuration table";
    case -47: return "Cannot open the directory where resource files are located";
    case -48: return "Open file failed";
    case -49: return "Motion parameter assignment failed";
    case -50: return "Node in Distributed mode";
    case -51: return "Homing failed";
    case -52: return "Distributed base processor failure";
    case -53: return "Maestro does not support this function or this working mode (NC Distributed)";
    case -54: return "Group contains less than 2 axes";
    case -55: return "Connection handler is incorrect";
    case -56: return "Incorrect motion operation mode";
    case -57: return "The communication type is not appropriate to this function";
    case -58: return "Wrong function argument type";
    case -59: return "Group is not found";
    case -60: return "Network scan failure";
    case -61: return "Network get statistics failure";
    case -62: return "Network reset statistics failure";
    case -63: return "FS (File System) failure";
    case -64: return "Dynamic memory allocation fail";
    case -65: return "XML parser failure";
    case -66: return "Open communication failure";
    case -67: return "Close communication failure";
    case -68: return "Communication scan bus process failure";
    case -69: return "An attempt was made to add an axis member to a group when the axis or ID is already present in the group";
    case -70: return "Max number of members in group is reached";
    case -72: return "Profiler recalc end velocity failure";
    case -74: return "Real time driver not initialized";
    default: return "Unknown error";
  }
}
