#pragma once

#ifdef CRBSCRIPT_EXPORTS
#define CRBSCRIPT_API __declspec(dllexport)
#else
#define CRBSCRIPT_API __declspec(dllimport)
//#pragma comment(lib, "CRBScript.lib")
#endif

#include "crbast.h"

namespace crb {
	//class CRBSCRIPT_API CCRBScript {
	//public:
	//	CCRBScript(void);
	//};

	class CRBSCRIPT_API ASTResult {
	public:
		bool success = true;
		ast::Program* program = nullptr; 

		~ASTResult();
		void clear();
	};

	CRBSCRIPT_API bool parse(const char *source, ASTResult& ast);

	//extern CRBSCRIPT_API int nCRBScript;
	//CRBSCRIPT_API int fnCRBScript(void);
}