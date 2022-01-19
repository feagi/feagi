
# Copyright 2019 The FEAGI Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import random
from pymongo.errors import ServerSelectionTimeoutError
from pymongo import MongoClient, DESCENDING, ASCENDING
from inf import runtime_data, settings


class MongoManagement:
    def __init__(self):

        # resolve host environment
        self.db_params = runtime_data.parameters['Database']
        self.port = int(self.db_params['mongodb_port'])
        if runtime_data.running_in_container:
            self.host = self.db_params['mongodb_container_host']
            print("Attempting to connect to MongoDb via the container")
            self.client = MongoClient(self.host, self.port, serverSelectionTimeoutMS=5000)
        else:
            self.host = self.db_params['mongodb_local_host']
            print("Attempting to connect to MongoDb via direct host access")
            self.client = MongoClient(self.host, self.port, serverSelectionTimeoutMS=1)

        try:
            self.client.server_info()
            self.db = self.client[self.db_params['mongodb_db']]
            self.collection_genome = self.db[self.db_params['mongodb_genomes']]
            self.collection_mnist = self.db[self.db_params['mongodb_mnist']]
            self.collection_test_stats = self.db[self.db_params['mongodb_stats']]
            self.collection_membrane_potentials = self.db[self.db_params['mongodb_potentials']]
            self.collection_neuron_activities = self.db[self.db_params['mongodb_neurons']]
            print(
                settings.Bcolors.OKGREEN + "Success: Connection to << MongoDb >> has been established." + settings.Bcolors.ENDC)
            print("Number of the available genomes in the Genome database is: ", self.collection_genome.count())
        except ServerSelectionTimeoutError:
            print(settings.Bcolors.RED + "ERROR: Cannot connect to << MongoDb >> Database" + settings.Bcolors.ENDC)

    def insert_test_stats(self, stats_data):
        self.collection_test_stats.insert_one(stats_data)

    def inset_membrane_potentials(self, membrane_potential_data):
        self.collection_membrane_potentials.insert_one(membrane_potential_data)

    def insert_neuron_activity(self, fcl_data):
        self.collection_neuron_activities.insert_one(fcl_data)

    def insert_genome(self, genome_data):
        self.collection_genome.insert_one(genome_data)

    def insert_mnist_entry(self, mnist_data):
        self.collection_mnist.insert_one(mnist_data)

    # def read_genome(self, genome_id):
    #
    #     return genome

    def latest_genome(self):
        db_output = self.collection_genome.find({}).sort("generation_date", DESCENDING).limit(1)
        return db_output[0]

    def mnist_(self, seq):
        pipeline = [
            {"$match": {"mnist_seq": seq}}
        ]
        mnist_seq_data = self.collection_mnist.aggregate(pipeline=pipeline)
        return mnist_seq_data

    def mnist_seq(self, mnist_type, seq):
        if mnist_type not in ['training', 'test']:
            print("ERROR: Invalid MNIST type!")
        else:
            mnist_seq_data = self.collection_mnist.find({"mnist_type": mnist_type, "mnist_seq": seq})
            return mnist_seq_data[0]

    def highest_fitness_genome(self):
        db_output = self.collection_genome.find({}).sort("fitness", DESCENDING).limit(1)
        return db_output[0]

    def genome_count(self):
        return self.collection_genome.count()

    def random_genome(self, n):
        pipeline = [
            {"$sample": {"size": n}}
        ]
        genome_list = self.collection_genome.aggregate(pipeline=pipeline)
        return genome_list

    def random_fit_genome(self, fitness_level):
        pipeline = [
            {"$match": {"fitness": {"$gt": fitness_level}}}
        ]
        genomes = self.collection_genome.aggregate(pipeline=pipeline)

        # Return a random genome from the list of fit genomes
        list_count = 0
        genome_list = []
        for item in genomes:
            list_count += 1
            genome_list.append(item)
        genome = genome_list[random.randrange(0, list_count, 1)]
        return genome

    def top_n_genome(self, n):
        # Assumption: Fitness array function is returning the list sorted with highest fit on top
        genome_list = self.fitness_array()
        genome_count = len(genome_list)
        python_list = []

        for i in range(min(n, genome_count)):
            python_list.append(genome_list[i])
        return python_list

    def random_m_from_top_n(self, selection_count, top_count):
        top_list = self.top_n_genome(top_count)
        python_list = []
        for _ in range(selection_count):
            python_list.append(top_list[random.randrange(0, len(top_list), 1)])
        return python_list

    def genome_id_2_properties(self, genome_id):
        genome = self.collection_genome.find_one({"genome_id": genome_id})
        return genome

    def fcl_data(self, genome_id):
        fcl_data = self.collection_neuron_activities.find({"genome_id": genome_id})
        return fcl_data

    def fitness_array(self):
        pipeline = [
            {"$match": {"fitness": {"$exists": True, "$ne": 0},
                        "genome_id": {"$exists": True, "$nin": [""]}}},
            {"$project": {"genome_id": 1, "fitness": 1}},
            {"$sort": {"fitness": -1}}
        ]
        fitness_list = self.genome_aggregate_function(pipeline)
        # results = self.collection_genome.aggregate(pipeline=pipeline)
        # fitness_list = []
        # for item in results:
        #     fitness_list.append(item)
        #     print(item)
        return fitness_list

    def genome_aggregate_function(self, pipeline):
        results = self.collection_genome.aggregate(pipeline=pipeline)
        python_list = self.mongo_2_list(results)
        return python_list

    def mongo_2_list(self, mongo_obj):
        python_list = []
        for item in mongo_obj:
            python_list.append(item)
        return python_list

    def id_list_2_genome_list(self, id_list):
        genome_list = []
        for _ in id_list:
            genome_list.append(self.genome_id_2_properties(_["genome_id"]))
        return genome_list

    def mnist_read_single_digit(self, mnist_type, seq, kernel):
        return self.collection_mnist.find({"mnist_type": mnist_type, 'mnist_seq': seq, 'kernel_size': kernel})[0]

    def mnist_read_nth_digit(self, mnist_type, n, kernel_size, digit):
        """
        // Requires official MongoShell 3.6+
        use metis;
        db.getCollection("mnist").find(
            {
                "digit" : "5",
                "kernel_size" : NumberInt(7),
                "mnist_type" : "training"
            }
        ).sort(
            {
                "mnist_seq" : 1.0
            }
        ).skip(1000).limit(1);
        """
        return self.collection_mnist.find({"mnist_type": mnist_type, 'digit': str(digit), 'kernel_size': kernel_size}).sort(
            "mnist_seq", 1).skip(n).limit(1)[0]

    def test_mongodb(self):
        try:
            dbs = self.client.list_database_names()
            if self.db_params['mongodb_db'] not in dbs:
                self.db = self.client[self.db_params['mongodb_db']]
            collections = self.db.list_collection_names()
            if self.db_params['mongodb_genomes'] not in collections:
                self.db.create_collection(self.db_params['mongodb_genomes'])
            print("    MongoDb: ", settings.Bcolors.OKGREEN + "<< Database and collections OK >>" + settings.Bcolors.ENDC)
        except Exception as e:
            print("    MongoDb:", settings.Bcolors.RED + str(e) + settings.Bcolors.ENDC)


class InfluxManagement:
    def __init__(self):
        import influxdb_client
        from influxdb_client.client.write_api import SYNCHRONOUS

        self.db_params = runtime_data.parameters['Database']

        if runtime_data.parameters:
            self.evo_bucket = self.db_params["influxdb_evolutionary_bucket"]
            self.stats_bucket = self.db_params["influxdb_stats_bucket"]
            self.org = self.db_params["influxdb_organization"]
            self.token = self.db_params["influxdb_token"]

            # todo: db address needs to be def from a config file instead
            print('Running in container: ', runtime_data.running_in_container)
            if runtime_data.running_in_container:
                self.url = self.db_params['influxdb_url']
            else:
                self.url = "http://127.0.0.1:8086"

            try:
                print("\n\n\nAttempting to connect to influxDb service on %s...\n\n\n" % self.url)
                self.client = influxdb_client.InfluxDBClient(
                    url=self.url,
                    token=self.token,
                    org=self.org
                )
                self.write_client = self.client.write_api(write_options=SYNCHRONOUS)
            except:
                print("ERROR: Influx service is not running!!!")
        else:
            print("ERROR: Parameters are not set for InfluxDb configuration!")

        # # check if running in a container
        # try:
        #     print(self.client.ping())
        #     host = self.container_host
        #
        # except ConnectionError:
        #     print("InfluxDB ERROR!!!")

        # try:
        #     print("Connected to Influxdb client %s on %s:%s" % (self.client.ping(), self.host, self.port))
        #     print(runtime_data.parameters["InitData"])
        #     print("Using %s as stat collection database" % self.stat_db)
        #     print("Using %s as evolutionary data collection database" % self.evo_db)
        #
        #     def db_existence_check(db_name):
        #         """Checks the existence of a database and creates it if it doesnt exist."""
        #         self.db_list = self.client.get_list_database()
        #         if db_name not in [db['name'] for db in self.db_list]:
        #             print("Creating InfluxDb database named ", db_name)
        #             self.client.create_database(db_name)
        #         else:
        #             print("InfluxDB %s exists" % db_name)
        #             return True
        #
        #     stat_db = db_existence_check(self.stat_db)
        #     evo_db = db_existence_check(self.evolutionary_database)
        #
        #     if not runtime_data.parameters["Database"]["influx_keep_stats"] and stat_db:
        #         self.client.drop_database(self.stat_db)
        #         print("Warning: Data from previous stat database was dropped!")
        #
        # except Exception as e:
        #     print(settings.Bcolors.RED +
        #           "ERROR: Cannot connect to << InfluxDb >> Database \n ::: %s" % str(repr(e)) + settings.Bcolors.ENDC)

    def insert_neuron_activity(self, connectome_path, cortical_area, neuron_id, membrane_potential):
        raw_data = [
            {
                "measurement": "membranePotential",
                "tags": {
                    "connectome": connectome_path,
                    "cortical_area": cortical_area,
                    "neuron": neuron_id
                },
                "fields": {
                    "membrane_potential": membrane_potential
                }
            }
        ]
        self.write_client.write(bucket=self.stats_bucket, org=self.org, record=raw_data)

    def insert_burst_activity(self, connectome_path, burst_id, cortical_area, neuron_count):
        raw_data = [
            {
                "measurement": "burstStats",
                "tags": {
                    "connectome": connectome_path,
                    "cortical_area": cortical_area,
                },
                "fields": {
                    "burst_id": burst_id,
                    "neuron_count": neuron_count
                }
            }
        ]
        self.write_client.write(bucket=self.stats_bucket, org=self.org, record=raw_data)

    def insert_burst_checkpoints(self, connectome_path, burst_id):
        raw_data = [
            {
                "measurement": "burstCheckpoints",
                "tags": {
                    "connectome": connectome_path
                },
                "fields": {
                    "burst_id": 1
                }
            }
        ]
        self.write_client.write(bucket=self.stats_bucket, org=self.org, record=raw_data)

    def insert_connectome_stats(self, connectome_path, cortical_area, neuron_count, synapse_count):
        raw_data = [
            {
                "measurement": "connectomeStats",
                "tags": {
                    "connectome": connectome_path,
                    "cortical_area": cortical_area,
                },
                "fields": {
                    "neuron_count": neuron_count,
                    "synapse_count": synapse_count
                }
            }
        ]
        self.write_client.write(bucket=self.stats_bucket, org=self.org, record=raw_data)

    def insert_inter_cortical_stats(self, connectome_path, cortical_area_src, cortical_area_dst, synapse_count):
        raw_data = [
            {
                "measurement": "interCorticalStats",
                "tags": {
                    "connectome": connectome_path,
                    "Source_cortical_area": cortical_area_src,
                    "Destination_cortical_area": cortical_area_dst
                },
                "fields": {
                    "synapse_count": synapse_count
                }
            }
        ]
        self.write_client.write(bucket=self.stats_bucket, org=self.org, record=raw_data)

    def insert_evolutionary_fitness_stats(self, connectome_path, fitness_score,
                                          training_sets, test_sets, training_exposure, test_exposure):
        raw_data = [
            {
                "measurement": "fitness",
                "tags": {
                    "connectome": connectome_path
                },
                "fields": {
                    "brain_fitness": fitness_score,
                    "training_sets": training_sets,
                    "test_sets": test_sets,
                    "training_exposure": training_exposure,
                    "test_exposure": test_exposure
                }
            }
        ]
        self.write_client.write(bucket=self.evo_bucket, org=self.org, record=raw_data)

    def insert_evolutionary_connectome_stats(self, connectome_path, cortical_area, neuron_count, synapse_count):
        raw_data = [
            {
                "measurement": "evolutionaryConnectomeStats",
                "tags": {
                    "connectome": connectome_path,
                    "cortical_area": cortical_area,
                },
                "fields": {
                    "neuron_count": neuron_count,
                    "synapse_count": synapse_count
                }
            }
        ]
        self.write_client.write(bucket=self.evo_bucket, org=self.org, record=raw_data)
        print(">>>>>--------->>>>--------->>> Evolutionary stats logged in influx")

    def drop_neuron_activity(self):
        self.client.buckets_api().delete_bucket(self.stats_bucket)

    def test_influxdb(self):
        try:
            if self.client.buckets_api().find_buckets(name=self.stats_bucket):
                print("    InfluxDb: ", settings.Bcolors.OKGREEN + "Enabled" + settings.Bcolors.ENDC)
        except Exception:
            runtime_data.parameters["Database"]["influxdb_enabled"] = False
            print("    InfluxDb:", settings.Bcolors.RED + "Disabled" + settings.Bcolors.ENDC)


if __name__ == "__main__":

    # from PUs import IPU_vision
    # from misc import disk_ops
    # disk_ops.load_parameters_in_memory()
    # from configuration import runtime_data
    # disk_ops.genome_handler("./connectome/")

    # mnist = IPU_vision.MNIST()
    # import numpy as np
    #
    # mongo = MongoManagement()
    #
    # for i in range(10):
    #     # results = mongo.mnist_read_single_digit(mnist_type='training', seq=60, kernel=3)
    #     results = mongo.mnist_read_nth_digit(mnist_type='test', n=i, kernel_size=3, digit=4)
    #
    #     image = results['original_image']
    #     npimage = np.array(image)
    #
    #     for _ in npimage:
    #         print(_)

    influx_client = InfluxManagement()

    influx_client.test_influxdb()

    # influx_client.insert_connectome_stats(connectome_path='/asf/fwef/ee',
    #                                  cortical_area='vision_v8',
    #                                  neuron_count=4000,
    #                                  synapse_count=10000)



    # print(mnist.read_image(1))








    # #
    # for _ in mongo.fitness_array():
    #     print(_)
    # #
    # print(mongo.highest_fitness_genome())
    # print(type(latest_genome))
    # print(latest_genome["properties"])
    # for _ in latest_genome:
    #     print(_)
    #
    # random_genome = mongo.random_genome(1)

    # for _ in mongo.fcl_data('2018-07-28_11:16:12_938349_ZDW0VQ_G'):
    #     print(">>>", _)

    # print(mongo.random_genome())

    # print(">", mongo.top_n_genome(2))
