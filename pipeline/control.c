#include "control.h"
#include "../core/isa_registry.h"

ControlSignals control_decode(const DecodedInstr *i, const InstructionHandler *h){
	(void)i;
	ControlSignals s = {
        .reg_write = 0,
        .wb_src    = WB_NONE, // Safely handles the custom enum type
        .mem_read  = 0,
        .mem_write = 0,
        .is_branch = 0,
        .is_jump   = 0,
        .is_system = 0,
        .uses_rs1  = 0,
        .uses_rs2  = 0
    };
	switch(h->format){
	
		case FMT_R:{
			s.reg_write=1;
			s.wb_src=WB_ALU;
			s.mem_read=0;
			s.mem_write=0;
			s.is_branch=0;
			s.is_jump=0;
			s.is_system=0;
			s.uses_rs1=1;
			s.uses_rs2=1;
			break;
		}
		case FMT_I:
		case FMT_I_SHAMT:{
			s.reg_write=1;
			s.wb_src=WB_ALU;
			s.mem_read=0;
			s.mem_write=0;
			s.is_branch=0;
			s.is_jump=0;
			s.is_system=0;
			s.uses_rs1=1;
			s.uses_rs2=0;
			break;
		}
		case FMT_U:{
			s.reg_write=1;
			s.wb_src=WB_ALU;
			s.mem_read=0;
			s.mem_write=0;
			s.is_branch=0;
			s.is_jump=0;
			s.is_system=0;
			s.uses_rs1=0;
			s.uses_rs2=0;
			break;
		}
		case FMT_I_LOAD:{
			s.reg_write=1;
			s.wb_src=WB_MEM;
			s.mem_read=1;
			s.mem_write=0;
			s.is_branch=0;
			s.is_jump=0;
			s.is_system=0;
			s.uses_rs1=1;
			s.uses_rs2=0;
			break;
		}
		case FMT_S:{
			s.reg_write=0;
			s.wb_src=WB_NONE;
			s.mem_read=0;
			s.mem_write=1;
			s.is_branch=0;
			s.is_jump=0;
			s.is_system=0;
			s.uses_rs1=1;
			s.uses_rs2=1;
			break;
		}
		case FMT_B:{
			s.reg_write=0;
			s.wb_src=WB_NONE;
			s.mem_read=0;
			s.mem_write=0;
			s.is_branch=1;
			s.is_jump=0;
			s.is_system=0;
			s.uses_rs1=1;
			s.uses_rs2=1;
			break;
		}
		case FMT_J:{
			s.reg_write=1;
			s.wb_src=WB_PC4;
			s.mem_read=0;
			s.mem_write=0;
			s.is_branch=0;
			s.is_jump=1;
			s.is_system=0;
			s.uses_rs1=0;
			s.uses_rs2=0;
			break;
		}
		case FMT_I_JALR:{
			s.reg_write=1;
			s.wb_src=WB_PC4;
			s.mem_read=0;
			s.mem_write=0;
			s.is_branch=0;
			s.is_jump=1;
			s.is_system=0;
			s.uses_rs1=1;
			s.uses_rs2=0;
			break;
		}
		case FMT_I_SYS:{
			s.reg_write=0;
			s.wb_src=WB_NONE;
			s.mem_read=0;
			s.mem_write=0;
			s.is_branch=0;
			s.is_jump=0;
			s.is_system=1;
			s.uses_rs1=0;
			s.uses_rs2=0;
			break;
		}
	}

	return s;

}
