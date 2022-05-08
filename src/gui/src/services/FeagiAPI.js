import axios from "axios";

const FEAGI_URL = "http://localhost:8000/v1";

const FeagiAPI = {
  async getBaselineSensory() {
    const response = await axios
      .get(`${FEAGI_URL}/feagi/feagi/gui_baseline/ipu`, {
        headers: {},
        params: {},
      })
      .catch((error) => {
        console.error(error);
        throw error;
      });
    return response.data;
  },

  async getBaselineMotor() {
    const response = await axios
      .get(`${FEAGI_URL}/feagi/feagi/gui_baseline/opu`, {
        headers: {},
        params: {},
      })
      .catch((error) => {
        console.error(error);
        throw error;
      });
    return response.data;
  },

  async getBaselineMorphology() {
    const response = await axios
      .get(`${FEAGI_URL}/feagi/feagi/gui_baseline/morphology`, {
        headers: {},
        params: {},
      })
      .catch((error) => {
        console.error(error);
        throw error;
      });
    return response.data;
  },

  async getBaselineCorticalGenes() {
    const response = await axios
      .get(`${FEAGI_URL}/feagi/feagi/gui_baseline/cortical-genes`, {
        headers: {},
        params: {},
      })
      .catch((error) => {
        console.error(error);
        throw error;
      });
    return response.data;
  },

  async postGenomeFile(genomeFile) {
    const response = await axios
      .post(`${FEAGI_URL}/feagi/genome/upload/file`, genomeFile, {
        headers: { "Content-Type": "multipart/form-data" },
        params: {},
      })
      .catch((error) => {
        console.error(error);
        throw error;
      });
    return response.data;
  },

  async postGenomeString(genomeString) {
    const response = await axios
      .post(`${FEAGI_URL}/feagi/genome/upload/string`, genomeString, {
        headers: {},
        params: {},
      })
      .catch((error) => {
        console.error(error);
        throw error;
      });
    return response.data;
  },
};

export default FeagiAPI;
