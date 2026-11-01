SIM   = iverilog -g2012
RTL   = rtl/dpwm.sv rtl/pid_ctrl.sv rtl/buck_ctrl.sv
MODEL = model/buck_plant.sv model/afe_model.sv
TB    = tb/tb_buck.sv
SCENS = main nocal calfail ocp short ovp uvp uvlo

.PHONY: all build run lint synth design plots spice clean $(SCENS)
all: run

build/sim.vvp: $(RTL) $(MODEL) $(TB)
	mkdir -p build results
	$(SIM) -Wall -s tb_buck -o $@ $(TB) $(RTL) $(MODEL)

$(SCENS): build/sim.vvp
	@vvp -n build/sim.vvp +SCEN=$@ | tee results/$@.log

run: build/sim.vvp
	@for s in $(SCENS); do vvp -n build/sim.vvp +SCEN=$$s > results/$$s.log; grep -E "RESULT" results/$$s.log | sed "s/^/$$s: /"; done

lint:
	verilator --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND --top-module buck_ctrl $(RTL)

synth:
	yosys -q -p "read_verilog -sv $(RTL); synth -top buck_ctrl -flatten; stat" | tee results/yosys_stat.log

design:
	python3 -m scripts.design_compensator

plots: run
	python3 -m scripts.plot_results

spice: run
	python3 scripts/spice_replay.py

clean:
	rm -rf build results/*.csv results/*.log
