#curl -X POST http://localhost:8000/detect -F "image=@test_image.jpg"
for image in *.jpg; do
  echo "Processing $image"
  curl -X POST http://localhost:8000/detect -F "image=@${image}"
  echo -e "\n"
done
