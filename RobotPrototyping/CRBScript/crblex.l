%option never-interactive
%option noyywrap
%option bison-bridge
/*%option debug*/
%option prefix="crb"
%option outfile="crblex.yy.cpp" header-file="crblex.yy.h"

%top {
#include "crbast.h"
using namespace crb;

#undef YY_NO_UNISTD_H
#define YY_NO_UNISTD_H
#define YYSTYPE ast::Node*
}

%{
#include "stdlib.h"
#include "crbgrammar.tab.h"
%}

D [[:digit:]]

%%
[[:space:]]+					/* ignore whitespace */
-?{D}*({D}|{D}\.|\.{D}){D}*		{ *yylval = new ast::RealLiteral(atof(yytext)); return T_REAL_LITERAL; }
[a-zA-Z_][a-zA-Z_0-9]*          { *yylval = new ast::Identifier(yytext); return T_IDENTIFIER; }
\"[^\"]*\"                      { *yylval = ast::StringLiteral::parse(yytext); return T_STRING_LITERAL; }
\{|\}|\;|\=|\.|\,|\[|\]         { return yytext[0]; }
.                               /* ignore errors for now */

%%
