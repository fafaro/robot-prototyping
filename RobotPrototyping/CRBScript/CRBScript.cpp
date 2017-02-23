#include "stdafx.h"
#include "CRBScript.h"
#include <cstdio>
#include <vector>
#include "crblex.yy.h"
#include "crbgrammar.tab.h"


extern "C" void crberror(char const *s) {
	fprintf(stderr, "CRB parser error: %s\n", s);
}

// The global result of the parse
extern crb::ast::Program* crbprog;

namespace crb {

	CRBSCRIPT_API bool parse(const char *source, ASTResult& ast) {
		// Copy input string to buffer, so that it is modifiable,
		// and appended with two null terminators for the sake of
		// Flex.
		std::vector<char> buffer(std::strlen(source) + 3);
		std::memcpy(&buffer[0], source, buffer.size() - 3);
		buffer[buffer.size() - 3] = '\0';
		buffer[buffer.size() - 2] = '\0';
		buffer[buffer.size() - 1] = 'X';

		// Point the Flex scanner to our buffer.
		crb_scan_buffer(&buffer[0], buffer.size() - 1);
		assert(buffer[buffer.size() - 1] == 'X');

		// Did it parse?
		auto errorCode = crbparse();
		assert(buffer[buffer.size() - 1] == 'X');
		if (errorCode) return false;

		assert(buffer[buffer.size() - 1] == 'X');

		// Return AST
		ast.clear();
		ast.program = crbprog;
		return true;
	}

	ASTResult::~ASTResult() {
		clear();
	}

	void ASTResult::clear() {
		if (program) {
			delete program;
			program = nullptr;
		}
	}
}


//CRBSCRIPT_API int nCRBScript=0;
//CRBSCRIPT_API int fnCRBScript(void)
//{
//    return 42;
//}
//
//CCRBScript::CCRBScript()
//{
//    return;
//}


