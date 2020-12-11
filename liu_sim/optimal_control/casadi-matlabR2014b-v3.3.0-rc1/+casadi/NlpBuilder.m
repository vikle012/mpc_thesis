classdef  NlpBuilder < casadi.PrintableCommon
    %NLPBUILDER A symbolic NLP representation.
    %
    %
    %
    %Joel Andersson
    %
    %C++ includes: nlp_builder.hpp 
    %
  methods
    function v = x(self)
      v = casadiMEX(962, self);
    end
    function v = f(self)
      v = casadiMEX(963, self);
    end
    function v = g(self)
      v = casadiMEX(964, self);
    end
    function v = x_lb(self)
      v = casadiMEX(965, self);
    end
    function v = x_ub(self)
      v = casadiMEX(966, self);
    end
    function v = g_lb(self)
      v = casadiMEX(967, self);
    end
    function v = g_ub(self)
      v = casadiMEX(968, self);
    end
    function v = x_init(self)
      v = casadiMEX(969, self);
    end
    function v = lambda_init(self)
      v = casadiMEX(970, self);
    end
    function v = discrete(self)
      v = casadiMEX(971, self);
    end
    function varargout = import_nl(self,varargin)
    %IMPORT_NL Import an .nl file.
    %
    %  IMPORT_NL(self, char filename, struct opts)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(972, self, varargin{:});
    end
    function varargout = type_name(self,varargin)
    %TYPE_NAME Readable name of the class.
    %
    %  char = TYPE_NAME(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(973, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP Print a description of the object.
    %
    %  std::ostream & = DISP(self, bool more)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(974, self, varargin{:});
    end
    function varargout = str(self,varargin)
    %STR Get string representation.
    %
    %  char = STR(self, bool more)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(975, self, varargin{:});
    end
    function self = NlpBuilder(varargin)
    %NLPBUILDER 
    %
    %  new_obj = NLPBUILDER()
    %
    %
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(976, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        casadiMEX(977, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
