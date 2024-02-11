{
    loaders: [{
      test: /\.jsx?$/,
      loaders: ['babel?retainLines=true'],
      include: path.join(__dirname, 'src')
    }]
}