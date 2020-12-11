function v = OP_PRINTME()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 93);
  end
  v = vInitialized;
end
