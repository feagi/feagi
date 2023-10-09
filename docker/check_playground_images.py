import docker
import requests


def get_image_digest(image_name, tag):
    client = docker.from_env()
    try:
        image = client.images.get(f"{image_name}:{tag}")
        digest = image.attrs['RepoDigests'][0]
        return digest
    except docker.errors.ImageNotFound:
        print(f"Image '{image_name}:{tag}' not found.")
    except Exception as e:
        print(f"An error occurred: {e}")


def pull_image(image_name, tag):
    print("Pulling ", image_name, ":" ,tag, " now....")
    client = docker.from_env()
    try:
        image = f"{image_name}:{tag}"
        client.images.pull(image)
        print(f"Successfully pulled image: {image}")
    except Exception as e:
        print(f"An error occurred: {e}")


def get_dockerhub_digest(url):
    response = requests.get(url)

    if response.status_code == 200:
        data = response.json()
        for result in data["results"]:
            if result["name"] == "latest":
                digest = result["digest"]
                return digest
    else:
        print(f"Error: Unable to fetch data. Status code {response.status_code}")


if __name__ == "__main__":
    image_list = ["neuraville/playground", "neuraville/webcam_controller", "neuraville/microbit",
                  "neuraville/feagi", "neuraville/bridge", "neuraville/brain_pong"]
    for current_image in image_list:
        name_of_image = current_image  # Replace with your desired image name
        tag_type = "latest"  # Replace with your desired tag
        repo_digest = get_image_digest(name_of_image, tag_type)
        url = "https://registry.hub.docker.com/v2/repositories/" + current_image + "/tags"
        sha_obtained = get_dockerhub_digest(url)
        latest_sha = sha_obtained[len("sha256:"):]
        if repo_digest:
            sha = repo_digest.split(':')[-1]
            if sha == latest_sha:
                print("your ", current_image, " is up to dated.")
        else:
            pull_image(name_of_image, tag_type)
