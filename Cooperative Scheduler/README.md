A lightweight cooperative multitasking scheduler implementation for STM32 microcontrollers, demonstrating real-time embedded systems concepts without requiring a full RTOS.
Features

Cooperative Multitasking: 
Tasks voluntarily yield control to maintain system responsiveness
Three Priority-Based Tasks: LED control, button handling, and idle processing
10ms System Tick: Precise timing using SysTick timer at 100Hz
Round-Robin Scheduling: Fair task execution with voluntary yield points
Interrupt-Driven Button Response: Immediate handling of user input via GPIO EXTI
Task Delay Mechanism: Non-blocking delays with automatic task wake-up
Scheduler Statistics: Performance monitoring and task switching metrics

Architecture
Task Control Block (TCB)
Each task is managed through a Task Control Block containing:

Task ID and state (READY, RUNNING, BLOCKED, SUSPENDED)
Priority level and function pointer
Wake time for delayed tasks
Human-readable task name

System Timing

SysTick Timer: 10ms interrupts (170MHz / 100Hz)
Schedule Flag: Set every tick to trigger scheduler operations
System Tick Counter: Global time reference for delays and timing.

