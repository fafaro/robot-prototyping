%require "3.0.4"
%defines "crbgrammar.tab.h"
%output "crbgrammar.tab.cpp"
%define api.value.type {ast::Node*}
%define api.pure full
%define parse.error verbose
%define api.prefix {crb}

%{
#include <stdio.h>
#include <math.h>
#include "crblex.yy.h"
#include "crbast.h"
using namespace crb;

extern "C" void crberror (char const *s);
extern ast::Program* crbprog = nullptr;
%}

%token T_STRING_LITERAL
%token T_REAL_LITERAL
%token T_IDENTIFIER
%token T_OPERATOR '{' '}' '.' '=' ';' ',' '[' ']'

%% /* Grammar rules and actions follow */

Program: 
	StatementList  { 
		$$ = new ast::Program(static_cast<ast::StatementList*>($1)); 
		crbprog = static_cast<ast::Program*>($$); 
	}
	;

StatementList:
	Statement {
		$$ = new ast::StatementList();
		auto slist = static_cast<ast::StatementList*>($$);
		auto stat = static_cast<ast::Statement*>($1);
		slist->collection.push_back(stat);
	}
	| StatementList Statement {
		$$ = $1;
		auto stat = static_cast<ast::Statement*>($2);
		static_cast<ast::StatementList*>($1)->collection.push_back(stat);
	}
	;

Statement:
	Expression ';' {
		$$ = new ast::Statement(static_cast<ast::Expression*>($1));
	}
	;

Expression:
	ObjectInstantiation { $$ = $1; }
	| TupleLiteral      { $$ = $1; }
	| T_STRING_LITERAL  { $$ = $1; }
	| T_REAL_LITERAL    { $$ = $1; }
	;

ObjectInstantiation:
	T_IDENTIFIER ClassInitializer {	$$ = new ast::ObjectLiteral($1, $2); }
	;

ClassInitializer:
	'{' PropertyAssignmentList OptionalComma '}' { $$ = new ast::ObjectInitializer($2);	}
	;

OptionalComma:
	','
	|
	;

PropertyAssignmentList:
	PropertyAssignment { $$ = new ast::PropertyAssignmentList($1); }
	| PropertyAssignmentList ',' PropertyAssignment {
		ast::PropertyAssignmentList::s_append($1, $3);
		$$ = $1;
	}
	| /* empty */ { $$ = new ast::PropertyAssignmentList(); }
	;

PropertyAssignment:
	'.' T_IDENTIFIER '=' Expression { $$ = new ast::PropertyAssignment($2, $4); }
	;

TupleLiteral:
	'[' ExpressionList OptionalComma ']' { $$ = new ast::TupleLiteral($2); }
	;

ExpressionList:
	Expression { $$ = new ast::ExpressionList($1); }
	| ExpressionList ',' Expression {
		ast::ExpressionList::s_append($1, $3);
		$$ = $1;
	}
	;

%%

