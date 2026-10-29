// Optional debug hooks for the ELEC1601 simulator.
//
// Copy this file next to your .ino and #include "SimHooks.h". On the real robot (no
// ELEC1601_SIM) every hook compiles to nothing, so the same sketch runs unchanged.
// In the simulator each hook appears in the event log at the exact virtual time.
//
//   SIM_EVENT("turn_start", 90);          // a named event with a number
//   SIM_STATE("navPhase", navPhase);      // a named state variable (logged when it changes)

#ifndef ELEC1601_SIM_HOOKS_H
#define ELEC1601_SIM_HOOKS_H

#ifdef ELEC1601_SIM
extern "C" __attribute__((import_module("arduino"), import_name("simEvent")))
void __sim_event(const char *name, long value);
extern "C" __attribute__((import_module("arduino"), import_name("simState")))
void __sim_state(const char *name, long value);
#define SIM_EVENT(name, value) __sim_event((const char *)(name), (long)(value))
#define SIM_STATE(name, value) __sim_state((const char *)(name), (long)(value))
#else
#define SIM_EVENT(name, value) ((void)0)
#define SIM_STATE(name, value) ((void)0)
#endif

#endif
