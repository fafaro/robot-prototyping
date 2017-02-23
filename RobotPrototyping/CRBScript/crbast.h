#pragma once

#include <string>
#include <list>
#include <cassert>

namespace crb {
	namespace ast {
		class Node
		{
		public:
			virtual ~Node() {}
		};

		class Expression : public Node
		{

		};

		class RealLiteral : public Expression
		{
		public:
			double value;
			RealLiteral(double value) : value(value) {}
			virtual ~RealLiteral() {}
		};

		class StringLiteral : public Expression
		{
		public:
			std::string value;
			StringLiteral() {}
			StringLiteral(const char *s) : value(s) {}
			virtual ~StringLiteral() {}
			static StringLiteral *parse(const char *s) {
				auto result = new StringLiteral();
				result->value = std::string(s + 1, s + strlen(s) - 1);
				return result;
			}
		};

		class Identifier : public Node
		{
		public:
			std::string name;
			Identifier(const char *name) : name(name) {}
			virtual ~Identifier() {}
		};

		class PropertyAssignment : public Node
		{
		public:
			std::string key;
			Expression* expr;

			PropertyAssignment(Node *identNode, Node *exprNode) {
				auto ident = dynamic_cast<Identifier*>(identNode);
				auto expr = dynamic_cast<Expression*>(exprNode);
				assert(ident != nullptr);
				assert(expr != nullptr);
				key = ident->name;
				this->expr = expr;
				delete ident;
			}
		};

		class PropertyAssignmentList : public Node
		{
		public:
			std::list<PropertyAssignment*> collection;

			PropertyAssignmentList() {}
			PropertyAssignmentList(Node *paNode) {
				auto pa = dynamic_cast<PropertyAssignment*>(paNode);
				assert(pa != nullptr);
				collection.push_back(pa);
			}
			static void s_append(Node *palNode, Node *paNode) {
				auto pal = dynamic_cast<PropertyAssignmentList*>(palNode);
				auto pa = dynamic_cast<PropertyAssignment*>(paNode);
				assert(pal != nullptr);
				assert(pa != nullptr);
				pal->collection.push_back(pa);
			}
		};

		class ObjectInitializer : public Node
		{
		public:
			PropertyAssignmentList* palist = nullptr;
			ObjectInitializer(Node *pal)
				: palist(dynamic_cast<decltype(palist)>(pal)) {

			}
			virtual ~ObjectInitializer() {}
		};

		class ObjectLiteral : public Expression
		{
		public:
			std::string typeName;
			PropertyAssignmentList *paList;

			ObjectLiteral(Node *identNode, Node *initializerNode) {
				auto ident = dynamic_cast<Identifier*>(identNode);
				auto initializer = dynamic_cast<ObjectInitializer*>(initializerNode);
				assert(ident != nullptr);
				assert(initializer != nullptr);
				typeName = ident->name;
				delete ident;

				paList = initializer->palist;
				initializer->palist = nullptr;
				delete initializer;
			}
			virtual ~ObjectLiteral() {}
		};


		class ExpressionList : public Node
		{
		public:
			std::list<Expression*> collection;

			ExpressionList(Node *exprNode) {
				auto expr = dynamic_cast<Expression*>(exprNode);
				assert(expr != nullptr);
				collection.push_back(expr);
			}

			static void s_append(Node *elistNode, Node *exprNode) {
				auto expr = dynamic_cast<Expression*>(exprNode);
				assert(expr != nullptr);
				auto elist = dynamic_cast<ExpressionList*>(elistNode);
				assert(elist != nullptr);
				elist->collection.push_back(expr);
			}
		};

		class TupleLiteral : public Expression
		{
		public:
			std::list<Expression*> collection;

			TupleLiteral(Node *exprListNode) {
				auto exprList = dynamic_cast<ExpressionList*>(exprListNode);
				assert(exprList != nullptr);
				collection = exprList->collection;
				exprList->collection.clear();
				delete exprListNode;
			}
		};



		class Statement : public Node
		{
		public:
			Expression *expr;
			Statement(Expression *expr) : expr(expr) {}
			virtual ~Statement() {}
		};

		class StatementList : public Node
		{
		public:
			std::list<Statement*> collection;
			virtual ~StatementList() {}
		};

		class Program : public Node
		{
		public:
			StatementList* slist = nullptr;

			Program(StatementList* slist) : slist(slist) {}
			virtual ~Program() {}
		};

	}
}