function v = OPTI_PAR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 135);
  end
  v = vInitialized;
end
