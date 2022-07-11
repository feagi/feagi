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

  async getBaselineMorphologyScalar() {
    const response = await axios
      .get(`${FEAGI_URL}/feagi/feagi/gui_baseline/morphology-scalar`, {
        headers: {},
        params: {},
      })
      .catch((error) => {
        console.error(error);
        throw error;
      });
    return response.data;
  },

  async getBaselinePscMultiplier() {
    const response = await axios
      .get(`${FEAGI_URL}/feagi/feagi/gui_baseline/psc-multiplier`, {
        headers: {},
        params: {},
      })
      .catch((error) => {
        console.error(error);
        throw error;
      });
    return response.data;
  },

  async getBaselinePlasticityFlag() {
    const response = await axios
      .get(`${FEAGI_URL}/feagi/feagi/gui_baseline/plasticity-flag`, {
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

  async postDefaultGenome(defaultGenome) {
    const response = await axios
      .post(`${FEAGI_URL}/feagi/genome/upload/default`, defaultGenome, {
        headers: {},
        params: {},
      })
      .catch((error) => {
        console.error(error);
        throw error;
      });
    return response.data;
  },

  async postGenomeFileEdit(genomeFile) {
    const response = await axios
      .post(`${FEAGI_URL}/feagi/genome/upload/file/edit`, genomeFile, {
        headers: { "Content-Type": "multipart/form-data" },
        params: {},
      })
      .catch((error) => {
        console.error(error);
        throw error;
      });
    return JSON.parse(response.data);
  },

  async getBurstCount() {
    const response = await axios
      .get(`${FEAGI_URL}/feagi/feagi/burst_engine/burst_counter`, {
        headers: {},
        params: {},
      })
      .catch((error) => {
        console.error(error);
        throw error;
      });
    return response.data;
  },

  async postShockRobot() {
    const response = await axios
      .post(
        `${FEAGI_URL}/feagi/training/shock`,
        { shock: true },
        {
          headers: {},
          params: {},
        }
      )
      .catch((error) => {
        console.error(error);
        throw error;
      });
    return response.data;
  },

  async postAutopilot() {
    const response = await axios
      .post(
        `${FEAGI_URL}/feagi/evolution/autopilot/on`,
        {},
        {
          headers: {},
          params: {},
        }
      )
      .catch((error) => {
        console.error(error);
        throw error;
      });
    return response.data;
  },

  async getDefaultGenomes() {
    const response = await axios
      .get(`${FEAGI_URL}/feagi/genome/defaults/files`, {
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
