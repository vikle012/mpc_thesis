classdef  MetaCon < casadi.IndexAbstraction
    %METACON 
    %
    %   = METACON()
    %
    %
  methods
    function v = original(self)
      v = casadiMEX(1129, self);
    end
    function v = canon(self)
      v = casadiMEX(1130, self);
    end
    function v = type(self)
      v = casadiMEX(1131, self);
    end
    function v = lb(self)
      v = casadiMEX(1132, self);
    end
    function v = ub(self)
      v = casadiMEX(1133, self);
    end
    function v = n(self)
      v = casadiMEX(1134, self);
    end
    function v = flipped(self)
      v = casadiMEX(1135, self);
    end
    function v = dual_canon(self)
      v = casadiMEX(1136, self);
    end
    function v = dual(self)
      v = casadiMEX(1137, self);
    end
    function v = extra(self)
      v = casadiMEX(1138, self);
    end
    function self = MetaCon(varargin)
    %METACON 
    %
    %  new_obj = METACON()
    %
    %
      self@casadi.IndexAbstraction(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1139, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        casadiMEX(1140, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
