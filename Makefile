# Top-level Makefile for NILM-TT (§10)

VERILOG_SRCS = \
	src/project.v \
	src/nilm_host_spi.v \
	src/nilm_cmd.v \
	src/nilm_csr.v \
	src/nilm_qspi.v \
	src/nilm_seq.v \
	src/nilm_mac.v \
	src/nilm_pwl.v \
	src/nilm_cell.v \
	src/nilm_head.v \
	src/nilm_vram.v

.PHONY: all lint golden test test-datapath synth clean

all: lint golden test test-datapath synth

lint:
	iverilog -g2005 -Isrc -o /tmp/nilm_lint.vvp $(VERILOG_SRCS)
	verilator --lint-only -Wall --top-module tt_um_harsh_nilm $(VERILOG_SRCS)

golden:
	pytest -v golden/

test:
	cd test && $(MAKE) clean && $(MAKE)
	test -s test/results.xml && ! grep -E '<(failure|error)' test/results.xml

test-datapath:
	cd test && $(MAKE) TOPLEVEL=nilm_mac MODULE=test_mac COCOTB_TEST_MODULES=test_mac SIM_BUILD=sim_build/mac COCOTB_RESULTS_FILE=results_mac.xml
	test -s test/results_mac.xml && ! grep -E '<(failure|error)' test/results_mac.xml
	cd test && $(MAKE) TOPLEVEL=nilm_cell MODULE=test_cell COCOTB_TEST_MODULES=test_cell SIM_BUILD=sim_build/cell COCOTB_RESULTS_FILE=results_cell.xml
	test -s test/results_cell.xml && ! grep -E '<(failure|error)' test/results_cell.xml

synth:
	yosys -p "read_verilog -Isrc $(VERILOG_SRCS); synth -top tt_um_harsh_nilm; stat"

clean:
	cd test && $(MAKE) clean
	rm -rf .pytest_cache golden/__pycache__ test/__pycache__ test/models/__pycache__
