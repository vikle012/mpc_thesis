classdef  Variable < casadi.PrintableCommon
    %VARIABLE 
    %
    %   = VARIABLE()
    %
    %
  methods
    function varargout = name(self,varargin)
    %NAME 
    %
    %  char = NAME(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(978, self, varargin{:});
    end
    function v = v(self)
      v = casadiMEX(979, self);
    end
    function v = d(self)
      v = casadiMEX(980, self);
    end
    function v = nominal(self)
      v = casadiMEX(981, self);
    end
    function v = start(self)
      v = casadiMEX(982, self);
    end
    function v = min(self)
      v = casadiMEX(983, self);
    end
    function v = max(self)
      v = casadiMEX(984, self);
    end
    function v = guess(self)
      v = casadiMEX(985, self);
    end
    function v = derivative_start(self)
      v = casadiMEX(986, self);
    end
    function v = variability(self)
      v = casadiMEX(987, self);
    end
    function v = causality(self)
      v = casadiMEX(988, self);
    end
    function v = category(self)
      v = casadiMEX(989, self);
    end
    function v = alias(self)
      v = casadiMEX(990, self);
    end
    function v = description(self)
      v = casadiMEX(991, self);
    end
    function v = valueReference(self)
      v = casadiMEX(992, self);
    end
    function v = unit(self)
      v = casadiMEX(993, self);
    end
    function v = display_unit(self)
      v = casadiMEX(994, self);
    end
    function v = free(self)
      v = casadiMEX(995, self);
    end
    function varargout = type_name(self,varargin)
    %TYPE_NAME 
    %
    %  char = TYPE_NAME(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(996, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP 
    %
    %  std::ostream & = DISP(self, bool more)
    %
    %
      [varargout{1:nargout}] = casadiMEX(997, self, varargin{:});
    end
    function varargout = str(self,varargin)
    %STR 
    %
    %  char = STR(self, bool more)
    %
    %
      [varargout{1:nargout}] = casadiMEX(998, self, varargin{:});
    end
    function self = Variable(varargin)
    %VARIABLE 
    %
    %  new_obj = VARIABLE()
    %  new_obj = VARIABLE(char name, Sparsity sp)
    %
    %
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(999, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        casadiMEX(1000, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
