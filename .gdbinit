target extended-remote :3333
file build/main.elf
load

# b main
# b mc_entry
# b runStateMachine
b runLEDThread

