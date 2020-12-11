classdef  OptiAdvanced < casadi.Opti
    %OPTIADVANCED C++ includes: optistack.hpp 
    %
    %
    %
  methods
    function delete(self)
      if self.swigPtr
        casadiMEX(1153, self);
        self.swigPtr=[];
      end
    end
    function varargout = solver(self,varargin)
    %SOLVER Get the underlying CasADi solver of the Opti stack.
    %
    %  Function = SOLVER(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1154, self, varargin{:});
    end
    function varargout = is_parametric(self,varargin)
    %IS_PARAMETRIC return true if expression is only dependant on Opti parameters, not
    %
    %  bool = IS_PARAMETRIC(self, MX expr)
    %
    %variables
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1155, self, varargin{:});
    end
    function varargout = symvar(self,varargin)
    %SYMVAR Get symbols present in expression.
    %
    %  {MX} = SYMVAR(self)
    %  {MX} = SYMVAR(self, MX expr)
    %  {MX} = SYMVAR(self, MX expr, casadi::VariableType type)
    %
    %
    %Returned vector is ordered according to the order of variable()/parameter()
    %calls used to create the variables
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1156, self, varargin{:});
    end
    function varargout = canon_expr(self,varargin)
    %CANON_EXPR Interpret an expression (for internal use only)
    %
    %  MetaCon = CANON_EXPR(self, MX expr)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1157, self, varargin{:});
    end
    function varargout = get_meta(self,varargin)
    %GET_META Get meta-data of symbol (for internal use only)
    %
    %  MetaVar = GET_META(self, MX m)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1158, self, varargin{:});
    end
    function varargout = get_meta_con(self,varargin)
    %GET_META_CON Get meta-data of symbol (for internal use only)
    %
    %  MetaCon = GET_META_CON(self, MX m)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1159, self, varargin{:});
    end
    function varargout = set_meta(self,varargin)
    %SET_META Set meta-data of an expression.
    %
    %  SET_META(self, MX m, MetaVar meta)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1160, self, varargin{:});
    end
    function varargout = set_meta_con(self,varargin)
    %SET_META_CON Set meta-data of an expression.
    %
    %  SET_META_CON(self, MX m, MetaCon meta)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1161, self, varargin{:});
    end
    function varargout = assert_active_symbol(self,varargin)
    %ASSERT_ACTIVE_SYMBOL 
    %
    %  ASSERT_ACTIVE_SYMBOL(self, MX m)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1162, self, varargin{:});
    end
    function varargout = active_symvar(self,varargin)
    %ACTIVE_SYMVAR 
    %
    %  {MX} = ACTIVE_SYMVAR(self, casadi::VariableType type)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1163, self, varargin{:});
    end
    function varargout = active_values(self,varargin)
    %ACTIVE_VALUES 
    %
    %  {DM} = ACTIVE_VALUES(self, casadi::VariableType type)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1164, self, varargin{:});
    end
    function varargout = x_lookup(self,varargin)
    %X_LOOKUP 
    %
    %  MX = X_LOOKUP(self, int i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1165, self, varargin{:});
    end
    function varargout = g_lookup(self,varargin)
    %G_LOOKUP 
    %
    %  MX = G_LOOKUP(self, int i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1166, self, varargin{:});
    end
    function varargout = x_describe(self,varargin)
    %X_DESCRIBE 
    %
    %  char = X_DESCRIBE(self, int i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1167, self, varargin{:});
    end
    function varargout = g_describe(self,varargin)
    %G_DESCRIBE 
    %
    %  char = G_DESCRIBE(self, int i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1168, self, varargin{:});
    end
    function varargout = describe(self,varargin)
    %DESCRIBE 
    %
    %  char = DESCRIBE(self, MX x, int indent)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1169, self, varargin{:});
    end
    function varargout = solve_prepare(self,varargin)
    %SOLVE_PREPARE 
    %
    %  SOLVE_PREPARE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1170, self, varargin{:});
    end
    function varargout = solve_actual(self,varargin)
    %SOLVE_ACTUAL 
    %
    %  struct:DM = SOLVE_ACTUAL(self, struct:DM args)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1171, self, varargin{:});
    end
    function varargout = arg(self,varargin)
    %ARG 
    %
    %  struct:DM = ARG(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1172, self, varargin{:});
    end
    function varargout = res(self,varargin)
    %RES 
    %
    %  RES(self, struct:DM res)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1173, self, varargin{:});
    end
    function varargout = constraints(self,varargin)
    %CONSTRAINTS 
    %
    %  {MX} = CONSTRAINTS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1174, self, varargin{:});
    end
    function varargout = objective(self,varargin)
    %OBJECTIVE 
    %
    %  MX = OBJECTIVE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1175, self, varargin{:});
    end
    function varargout = baked_copy(self,varargin)
    %BAKED_COPY 
    %
    %  OptiAdvanced = BAKED_COPY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1176, self, varargin{:});
    end
    function varargout = assert_empty(self,varargin)
    %ASSERT_EMPTY 
    %
    %  ASSERT_EMPTY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1177, self, varargin{:});
    end
    function varargout = bake(self,varargin)
    %BAKE Fix the structure of the optimization problem.
    %
    %  BAKE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1178, self, varargin{:});
    end
    function v = problem_dirty_(self)
      v = casadiMEX(1179, self);
    end
    function varargout = mark_problem_dirty(self,varargin)
    %MARK_PROBLEM_DIRTY 
    %
    %  MARK_PROBLEM_DIRTY(self, bool flag)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1180, self, varargin{:});
    end
    function varargout = problem_dirty(self,varargin)
    %PROBLEM_DIRTY 
    %
    %  bool = PROBLEM_DIRTY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1181, self, varargin{:});
    end
    function v = solver_dirty_(self)
      v = casadiMEX(1182, self);
    end
    function varargout = mark_solver_dirty(self,varargin)
    %MARK_SOLVER_DIRTY 
    %
    %  MARK_SOLVER_DIRTY(self, bool flag)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1183, self, varargin{:});
    end
    function varargout = solver_dirty(self,varargin)
    %SOLVER_DIRTY 
    %
    %  bool = SOLVER_DIRTY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1184, self, varargin{:});
    end
    function v = solved_(self)
      v = casadiMEX(1185, self);
    end
    function varargout = mark_solved(self,varargin)
    %MARK_SOLVED 
    %
    %  MARK_SOLVED(self, bool flag)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1186, self, varargin{:});
    end
    function varargout = solved(self,varargin)
    %SOLVED 
    %
    %  bool = SOLVED(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1187, self, varargin{:});
    end
    function varargout = assert_solved(self,varargin)
    %ASSERT_SOLVED 
    %
    %  ASSERT_SOLVED(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1188, self, varargin{:});
    end
    function varargout = assert_baked(self,varargin)
    %ASSERT_BAKED 
    %
    %  ASSERT_BAKED(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1189, self, varargin{:});
    end
    function varargout = instance_number(self,varargin)
    %INSTANCE_NUMBER 
    %
    %  int = INSTANCE_NUMBER(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1190, self, varargin{:});
    end
    function self = OptiAdvanced(varargin)
    %OPTIADVANCED 
    %
    %  new_obj = OPTIADVANCED(Opti x)
    %
    %
      self@casadi.Opti(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1191, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
  end
  methods(Static)
  end
end
