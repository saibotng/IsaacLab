import sys
from urllib.parse import urlparse
import boto3
import os


def upload_to_aws(local_dir: str, bucket_name: str, prefix: str) -> None:
    """
    Uploads a file to an AWS S3 bucket.

    Args:
        local_file_path (str): Path to the local file to upload.
        bucket_name (str): Name of the target S3 bucket.
        s3_file_path (str): Path in the S3 bucket where the file will be stored.
    """
    s3_client = boto3.client('s3')
    for root, _, files in os.walk(local_dir):
        for filename in files:
            local_path = os.path.join(root, filename)
            # preserve relative path inside S3 prefix
            relative_path = os.path.relpath(local_path, start=local_dir)
            s3_key = os.path.join(prefix, relative_path).replace("\\", "/")  # Windows safe
            print(f"Uploading {local_path} → s3://{bucket_name}/{s3_key}")
            s3_client.upload_file(local_path, bucket_name, s3_key)


def upload_to_aws_uri(local_path, s3_uri):
    if not s3_uri.startswith("s3://"):
        raise ValueError("s3_uri must start with 's3://'")
    
    parsed = urlparse(s3_uri)
    bucket_name = parsed.netloc
    s3_file_path = parsed.path.lstrip('/')

    upload_to_aws(local_path, bucket_name, s3_file_path)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python push_to_aws.py <local_file_path> <s3_uri>")
        sys.exit(1)

    local_dir_path = sys.argv[1]
    s3_uri = sys.argv[2]
    upload_to_aws_uri(local_dir_path, s3_uri)
