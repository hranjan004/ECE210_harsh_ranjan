# Top-level Makefile for NILM-TT (§10)
export PATH := /opt/homebrew/bin:/usr/local/bin:$(PATH)

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

.PHONY: all lint golden test synth clean

all: lint golden test synth

lint:
	iverilog -g2005 -Isrc -o /tmp/nilm_lint.vvp $(VERILOG_SRCS)
	verilator --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSED -Isrc --top-module tt_um_harsh_nilm $(VERILOG_SRCS)

golden:
	pytest -v golden/

test:
	cd test && $(MAKE) clean && $(MAKE)
	! grep failure test/results.xml

synth:
	yosys -p "read_verilog -Isrc $(VERILOG_SRCS); synth -top tt_um_harsh_nilm; stat"

clean:
	cd test && $(MAKE) clean
	rm -rf .pytest_cache golden/__pycache__ test/__pycache__ test/models/__pycache__
