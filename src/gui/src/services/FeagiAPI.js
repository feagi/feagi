import axios from "axios";

// const FEAGI_URL = "http://localhost:8000/v1";

const FEAGI_URL = window._env_.FEAGI_URL + "/v1";
console.log(FEAGI_URL);


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

  async postShockRobot(shock_scenarios) {
    const response = await axios
      .post(
        `${FEAGI_URL}/feagi/training/shock/activate`,
        shock_scenarios,
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

  async postRobotModel(model_data) {
    const response = await axios
      .post(
        `${FEAGI_URL}/robot/model`,
         model_data,
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

  async resetGenome() {
    const response = await axios
      .post(
        `${FEAGI_URL}/feagi/genome/reset`,
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

  async getShockOptions() {
    const response = await axios
      .get(`${FEAGI_URL}/feagi/training/shock/options`, {
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
