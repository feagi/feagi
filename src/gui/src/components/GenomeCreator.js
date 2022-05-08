import React, { useContext } from "react";
import SensoryContext from "../contexts/SensoryContext";

const GenomeCreator = () => {
  const test = useContext(SensoryContext);
  console.log("+++++++++++++++++++++++++++++++");
  console.log(test);
  return <div>{test}</div>;
};

export default GenomeCreator;
